/*
 * Log.c
 *
 *  Created on: May 13, 2023
 *      Author: harsha
 *
 *  FIX (2026-04-28):
 *  Log_DoOneCycle() previously used an else-if chain that gave cylinders
 *  absolute priority over motor logging.  Because the two cylinders almost
 *  always have new data, the motor branch was almost never reached, causing
 *  a ~13-42x imbalance (C entries vs D entries in the log).
 *
 *  Fix: check cylinders and motors independently in every call so that
 *  all channels are logged at comparable rates.
 *  The buffer-space guard is applied individually to each sub-packet so
 *  we never overrun even when all three slots are written in one call.
 */

#include "Log.h"

extern userSettings u;

uint8_t LOG_checkNewData(RunTime_TypeDef *r, Log *l, uint8_t motorID){
	if (r->rdngNo != l->mLog[motorID].loggedRdngNo){
		return 1;
	}
	return 0;
}

uint8_t Log_addDataToBuffer(RunTime_TypeDef *r, uint16_t bufferLocation, uint8_t motorID){
	sprintf(LogBuffer + bufferLocation,
	        "D,%01d,%05d,%04d,%04d,%04d,%05.02f,%06.02f,%03d,%03d,%05d,E\r\n",
	        motorID, r->rdngNo, r->targetRPM, r->presentRPM, r->pwm,
	        r->currentA, r->power, r->motorTemp, r->mosfetTemp, r->motorError);
	return PACKET_SIZE_NORMAL_MOTOR;
}

uint8_t Log_addSettingsDataToBuffer(userSettings *u, uint16_t bufferLocation){
	sprintf(LogBuffer + bufferLocation,
	        "S,%05.02f,%05.02f,%04d,%04d,%04d,%04d,%04d,E\r\n",
	        u->delivery_mMin, u->deliveryMtrMin_CardFeed_Ratio,
	        u->cardCylRPM, u->btrCylRPM, u->pickerCylRPM,
	        u->btrFeedRPM, u->AF_FeedRPM);
	return PACKET_SIZE_SETTINGS;
}

uint8_t Log_StateChangeDataToBuffer(StateTypeDef *s, uint16_t bufferLocation){
	sprintf(LogBuffer + bufferLocation,
	        "R,%02d,%01d,%01d,%06lu,E\r\n",
	        s->current_state, s->runMode, s->BT_pauseReason, s->oneSecTimer);
	return PACKET_SIZE_RUNSTATE;
}

/* Returns the next motor (from motorList) that has motorRunning == 1.
 * Wraps around. If only one motor is running it returns that same motor. */
uint8_t Log_changeLoggingMotor(Log *l){
	uint8_t currentMotor = l->loggingMotor;

	while (1){
		currentMotor = currentMotor + 1;
		if (currentMotor > 7){
			currentMotor = 0;
		}
		if (l->mLog[currentMotor].motorRunning == 1){
			break;
		}
		if (currentMotor == l->loggingMotor){
			break;
		}
	}

	return currentMotor;
}

void Log_setUpLogging(Log *l, uint8_t *motorList, uint8_t noOfMotors){
	Log_disableLogging(l);
	for (int i = 0; i < noOfMotors; i++){
		l->mLog[motorList[i]].motorRunning = 1;
		l->mLog[motorList[i]].newData      = 0;
		l->mLog[motorList[i]].loggedRdngNo = 0;
	}
	L.loggingMotor    = motorList[0];
	L.DMA_transferOver = 1;
}

void Log_disableLogging(Log *l){
	for (int i = 0; i < 8; i++){
		l->mLog[i].motorRunning = 0;
		l->mLog[i].newData      = 0;
		l->mLog[i].loggedRdngNo = 0;
	}
}

uint8_t Log_addCylDataToBuffer(ExtendedRunTime_TypeDef *er, uint16_t bufferLocation, uint8_t motorID){
	sprintf(LogBuffer + bufferLocation,
	        "C,%01d,%05d,%05d,%05d,%05.02f,%06.02f,%03d,%03d,%04x,%01d,E\r\n",
	        motorID, er->rdngNo, er->targetRPM, er->actualRPM,
	        er->peakPhaseCurrentApk, (float)er->power,
	        er->FETTemp, er->MOTTemp, er->currFault, er->motorState);
	return PACKET_SIZE_CYLINDER;
}

void Log_setUpCylinderLogging(Log *l){
	l->cylLoggedRdngNo[0] = 0;
	l->cylLoggedRdngNo[1] = 0;
	l->logCylinder = 1;
}


/* ---------------------------------------------------------------------------
 * Log_DoOneCycle  –  FIXED VERSION
 *
 * OLD behaviour (bug):
 *   if (cyl0 new data)       → log cyl0
 *   else if (cyl1 new data)  → log cyl1
 *   else                     → log one motor   ← almost never reached!
 *
 * NEW behaviour:
 *   Check cyl0  independently  → log if new data & buffer has space
 *   Check cyl1  independently  → log if new data & buffer has space
 *   Check motor independently  → log if new data & buffer has space
 *
 *   All three are attempted every call.  Motors are therefore logged at
 *   the same cadence as cylinders instead of being starved.
 * --------------------------------------------------------------------------*/
void Log_DoOneCycle(void){

	if (L.DMA_transferOver == 0){
		return;   // DMA still busy – nothing to do
	}

	/* ---- high-priority one-shot events (same as before) ---- */
	if (L.logRunStateChange == 1){
		if ((BUFFER_LOG_SIZE - L.bufferIdx) > PACKET_SIZE_RUNSTATE){
			L.bufferIdx += Log_StateChangeDataToBuffer(&S, L.bufferIdx);
			L.logRunStateChange = 0;
		}
		return;   // give the state-change packet its own DMA slot
	}

	if (L.logSettings == 1){
		if ((BUFFER_LOG_SIZE - L.bufferIdx) > PACKET_SIZE_SETTINGS){
			L.bufferIdx += Log_addSettingsDataToBuffer(&u, L.bufferIdx);
			L.logSettings = 0;
		}
		return;
	}

	/* ---- flush (idle / error mode) ---- */
	if (L.flushBuffer){
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)LogBuffer, L.bufferIdx);
		L.DMA_transferOver = 0;
		L.bufferIdx  = 0;
		L.flushBuffer = 0;
		return;
	}

	/* ---- normal run: log cylinders AND motors in parallel ---- */
	if (L.logCylinder == 1){

		/* --- Cylinder 0 (CARDING CYLINDER) --- */
		if (ER[0].rdngNo != L.cylLoggedRdngNo[0]){
			if ((BUFFER_LOG_SIZE - L.bufferIdx) > PACKET_SIZE_CYLINDER){
				L.bufferIdx += Log_addCylDataToBuffer(&ER[0], L.bufferIdx, 0);
				L.cylLoggedRdngNo[0] = ER[0].rdngNo;
			}
		}

		/* --- Cylinder 1 (BEATER CYLINDER) --- independent check, NOT else-if */
		if (ER[1].rdngNo != L.cylLoggedRdngNo[1]){
			if ((BUFFER_LOG_SIZE - L.bufferIdx) > PACKET_SIZE_CYLINDER){
				L.bufferIdx += Log_addCylDataToBuffer(&ER[1], L.bufferIdx, 1);
				L.cylLoggedRdngNo[1] = ER[1].rdngNo;
			}
		}

		/* --- Regular motors --- independent check, NOT else */
		L.mLog[L.loggingMotor].newData = LOG_checkNewData(&R[L.loggingMotor], &L, L.loggingMotor);
		if (L.mLog[L.loggingMotor].newData){
			if ((BUFFER_LOG_SIZE - L.bufferIdx) > PACKET_SIZE_NORMAL_MOTOR){
				L.bufferIdx += Log_addDataToBuffer(&R[L.loggingMotor], L.bufferIdx, L.loggingMotor);
				L.mLog[L.loggingMotor].loggedRdngNo = R[L.loggingMotor].rdngNo;
				L.loggingMotor = Log_changeLoggingMotor(&L);
			}
		}

	}
	else{
		/* Cylinder logging disabled – motor-only path (unchanged) */
		L.mLog[L.loggingMotor].newData = LOG_checkNewData(&R[L.loggingMotor], &L, L.loggingMotor);
		if (L.mLog[L.loggingMotor].newData){
			if ((BUFFER_LOG_SIZE - L.bufferIdx) > PACKET_SIZE_NORMAL_MOTOR){
				L.bufferIdx += Log_addDataToBuffer(&R[L.loggingMotor], L.bufferIdx, L.loggingMotor);
				L.mLog[L.loggingMotor].loggedRdngNo = R[L.loggingMotor].rdngNo;
				L.loggingMotor = Log_changeLoggingMotor(&L);
			}
		}
	}

	/* ---- flush when buffer is nearly full ---- */
	if ((BUFFER_LOG_SIZE - L.bufferIdx) <= PACKET_SIZE_MINIMUM){
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)LogBuffer, L.bufferIdx);
		L.DMA_transferOver = 0;
		L.bufferIdx = 0;
	}
}


void Log_ResetRunTimeRdngNos(void){
	for (int i = 0; i < 8; i++){
		R[i].rdngNo = 0;
	}
}

void Log_ResetBufferIndex(Log *l){
	l->bufferIdx = 0;
}

/*
 * RunState.c
 *
 *  Created on: 15-Apr-2023
 *      Author: harsha
 *
 */

#include "stdio.h"
#include "MachineErrors.h"
#include "StateMachine.h"
#include "BT_Fns.h"
#include "CommonConstants.h"
#include "CAN_MotherBoard.h"
#include "MotorComms.h"
#include "Ack.h"
#include "userButtons.h"
#include "SysObserver.h"
#include "TowerLamp.h"
#include "mcp23017.h"
#include "MachineSensors.h"
#include "Log.h"
#include "BT_Machine.h"
#include "TD_Pot.h"

extern UART_HandleTypeDef huart1;
uint8_t updateSettings = 0;


uint8_t startBtrFeed= 0,stopBtrFeed =0;
uint8_t sentMsgToBtrFeed = 0,BtrFeedMsgSentSuccess =0,BtrFeedMsgSentFail=0;
uint16_t beaterMotorRPM1 = 0;
uint8_t msgSuccess = 0;


void RunState(void){

	uint8_t response = 0;
	uint8_t noOfMotors = 0;
	uint8_t BTpacketSize = 0;
	long currentTime;
	while(1){

		if (S.oneTime){
			//send the start commands
			uint8_t motors[] = {CARDING_CYLINDER,BEATER_CYLINDER,AF_PICKER_CYLINDER};
			noOfMotors = 3;
			response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);// TODO: every response has to be handled !!

			S.runMode = RUN_RAMPUP;

			//when u start your in the run mode
			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_ON,AMBER_OFF);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);

			S.TD_POT_check = 0; // make this zero when u come into Run state
			S.oneTime = 0;
		}

		/*---------------- Beater Feed and AF Feed Logic -----------------*/

//		tgCoiler.currentReading = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,TG_OPTICAL_SENSOR);
//		tgCoiler.ductStateChanged = SensorAppyHysteresis(&tgCoiler);


		SensorCheckDeadTimeOver(&ductCardFeed);
		if (ductCardFeed.deadTimeOn==0){
			ductCardFeed.currentReading = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_TOP_CARDFEED);
			ductCardFeed.ductStateChanged = SensorAppyHysteresis(&ductCardFeed);

			if (ductCardFeed.ductStateChanged){
				C.D.cardFeed_sensorState = ductCardFeed.presentState; // putting the sensor states in the C struct
				if (S.runMode != RUN_RAMPUP){
					if (C.D.cardFeed_sensorState == CARD_DUCT_SENSOR_OPEN){
						msgSuccess = sendStartStopToBeaterFeedMotor(&C,START);
						C.D.cardFeed_ductState_toApp = DUCT_OPEN; // this is the var that goes to the app
						if (msgSuccess == 1){ // if success start the dead time calculation so that we dont send another command to the motor immediately
							SensorStartDeadTime(&ductCardFeed);
						}
						else{ // if not success, reset the sensor state to that the msg can go again.
							ductCardFeed.presentState = DUCT_SENSOR_RESET;
						}
					}
					if (C.D.cardFeed_sensorState == CARD_DUCT_SENSOR_CLOSED){
						msgSuccess = sendStartStopToBeaterFeedMotor(&C,RAMPDOWN_STOP);
						C.D.cardFeed_ductState_toApp = DUCT_CLOSED; // this is the var that goes to the app
						if (msgSuccess == 1){// if success start the dead time calculation so that we dont send another command to the motor immediately
							SensorStartDeadTime(&ductCardFeed);
						}
						else{ // if not success, reset the sensor state to that the msg can go again.
							ductCardFeed.presentState = DUCT_SENSOR_RESET;
						}
					}
				}
				ductCardFeed.ductStateChanged = 0;
			}
		} // closes deadTime

		// check duct auto Feed Status. Status is set here, and used in the Run State blocks of code below.
		SensorCheckDeadTimeOver(&ductAutoFeed);
		if (ductAutoFeed.deadTimeOn==0){
			ductAutoFeed.currentReading = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_AF);
			ductAutoFeed.ductStateChanged = SensorAppyHysteresis(&ductAutoFeed);
			if (ductAutoFeed.ductStateChanged){
				C.D.autoFeed_sensorState = ductAutoFeed.presentState;
				if (S.runMode != RUN_RAMPUP){
					if (C.D.autoFeed_sensorState == AFDUCT_SENSOR_OPEN){
						C.D.autoFeed_ductState_toApp = DUCT_OPEN; // this is the var that goes to the app
						msgSuccess = sendStartStopToAutoFeedMotor(&C,START);
						if (msgSuccess == 1){
							SensorStartDeadTime(&ductAutoFeed);
						}
						else{// if not success, reset the sensor state to that the msg can go again.
							ductAutoFeed.presentState = DUCT_SENSOR_RESET;
						}
					}else if (C.D.autoFeed_sensorState == AFDUCT_SENSOR_CLOSED){
						C.D.autoFeed_ductState_toApp = DUCT_CLOSED; // app var
						msgSuccess = sendStartStopToAutoFeedMotor(&C,RAMPDOWN_STOP);
						if (msgSuccess == 1){
							SensorStartDeadTime(&ductAutoFeed);
						}
						else{
							ductAutoFeed.presentState = DUCT_SENSOR_RESET; // if not success, reset the sensor state to that the msg can go again.
						}
					}
				}
				ductAutoFeed.ductStateChanged = 0;
			}
		}//closes deadtime


		/*----------------Ending Beater Feed and AF Feed Logic -----------------*/

		//--------run through the various Modes-----------------------------

		if (S.runMode == RUN_RAMPUP){
			C.L.stateMc_rampOver = CheckCylindersRampUpOver(&C,&ER[CARDING_CYLINDER],&ER[BEATER_CYLINDER],&R[AF_PICKER_CYLINDER]);
			if (C.L.stateMc_rampOver == 1){
				S.runMode = RUN_FILL_DUCT;
				//force the cardFeed section above to check the duct state by setting present state to not what it currently is
				ductCardFeed.presentState = DUCT_SENSOR_RESET;
				ductAutoFeed.presentState = DUCT_SENSOR_RESET;
			}
		}

		if (S.runMode == RUN_FILL_DUCT){
			if (C.D.cardFeed_sensorState == CARD_DUCT_SENSOR_CLOSED){
				if (usrBtns.rotarySwitch == ROTARY_SWITCH_ON){
					S.piecingMode = 1;
					updateCardingSectionPiecingSpeeds(&C,&u,I.piecingDeliveryMtrsMin);
				}else{
					S.piecingMode = 0;
					updateCardingSectionSpeeds(&C,&u);
				}
				ReadySetupRPMCommand_CardingMotors(&C);
				S.runMode = RUN_CARDING_SECTION;
				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				noOfMotors = 3;
				response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
			}
		}

		//piecing mode is only applied in runAll
		if (usrBtns.rotarySwitch == ROTARY_SWITCH_ON){
			if (S.piecingMode == 0){
				S.piecingMode = 1;
				updateCardingSectionPiecingSpeeds(&C,&u,I.piecingDeliveryMtrsMin);
				ReadySetupRPMCommand_CardingMotors(&C);

				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				uint16_t targets[] = {C.M.cardFeedMotorRPM,
						C.M.cageMotorRPM,C.M.coilerMotorRPM};
				noOfMotors = 3;
				SendChangeTargetToMultipleMotors(motors,noOfMotors,targets);
				S.piecingMode = 1;
				L.logRunStateChange = 1;
			}
		}else{
			if (S.piecingMode == 1){
				updateCardingSectionSpeeds(&C,&u);
				ReadySetupRPMCommand_CardingMotors(&C);

				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				uint16_t targets[] = {C.M.cardFeedMotorRPM,
						C.M.cageMotorRPM,C.M.coilerMotorRPM};
				noOfMotors = 3;
				SendChangeTargetToMultipleMotors(motors,noOfMotors,targets);

				//we send the default coiler speed here, so to force the code to check the
				//pot we reset the pot applied level here. this will force the tdp code to check
				// the pot reading and if its different from the applied level will apply it.
				tdp.appliedLevel = 0;

				S.piecingMode = 0;
				L.logRunStateChange = 1;
			}
		}



		/*if settings modified through app for carding:
		 * update the settings whatever the state(pause/rampup/fill or normal). But onyl send the change
		 * target during the normal mode , when we re not in piecing.
		 * but if in pause or rampup mode since the motors are not in run state,
		 * sending a change target wont do anything. Instead when we come out of
		 * the state, the start command to the motors will take into account the new settings
		 * In run state, only if we re not in piecing mode send the new settings.
		 * However the app will say settings updated, (even if it doesnt immediately apply)
		 */
		if (S.settingsModified){
			updateCardingSectionSpeeds(&C,&u);
			if ((S.runMode == RUN_CARDING_SECTION)&&(S.piecingMode==0)){
					uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
					uint16_t targets[] = {C.M.cardFeedMotorRPM,C.M.cageMotorRPM,C.M.coilerMotorRPM};
					noOfMotors = 3;
					SendChangeTargetToMultipleMotors(motors,noOfMotors,targets);
				}
			/*if ((S.runMode == RUN_CARDING_SECTION || S.runMode == RUN_FILL_DUCT)){
				updateFeedSectionSpeeds(&C,&u);
				ductCardFeedTop.presentState = DUCT_SENSOR_RESET; // resetting this state, forces the duct code to send a new command
				ductCardFeedBtm.presentState = DUCT_SENSOR_RESET;
			} DOESNT WORK!*/

			S.settingsModified = 0;
		}


	   if (S.runMode == RUN_CARDING_SECTION){
			// TO DO: Pieceing, tension draft changing,in pause mode changing settings..
			if (usrBtns.yellowBtn == BTN_PRESSED){
				usrBtns.yellowBtn = BTN_IDLE;
				//Pause
				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				noOfMotors = 3;
				response = SendCommands_To_MultipleMotors(motors,noOfMotors,RAMPDOWN_STOP);
				TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_OFF,AMBER_ON);
				TowerLamp_ApplyState(&hmcp,&mcp_portB);

				S.runMode = RUN_PAUSED;
				S.BT_pauseReason = BT_PAUSE_USER_PRESSED;
			}
		}

		if (S.runMode == RUN_PAUSED){
			if (usrBtns.greenBtn == BTN_PRESSED){
				usrBtns.greenBtn = BTN_IDLE;
				//RESUME
				if (usrBtns.rotarySwitch == ROTARY_SWITCH_ON){
					S.piecingMode = 1;
					updateCardingSectionPiecingSpeeds(&C,&u,I.piecingDeliveryMtrsMin);
				}else{
					S.piecingMode = 0;
					updateCardingSectionSpeeds(&C,&u);
					//we send the default coiler speed here, so to force the code to check the
					//pot we reset the pot applied level here. this will force the tdp code to check
					// the pot reading and if its different from the applied level will apply it.
					tdp.appliedLevel = 0;
				}
				ReadySetupRPMCommand_CardingMotors(&C);
				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				noOfMotors =3;
				response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);

				S.runMode = RUN_CARDING_SECTION;

				TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_ON,AMBER_OFF);
				TowerLamp_ApplyState(&hmcp,&mcp_portB);
			}
		}

		/*---- go into settings when need be-----*/
		if (S.switchState == TO_SETTINGS){
			ChangeState(&S,SETTINGS_STATE);
			S.switchState = 0;
			break;
		}

		if (S.oneSecTimer != currentTime){
			C.L.mcPower = ER[0].power + ER[1].power + R[2].power + R[3].power + R[4].power + R[5].power + R[6].power+ R[7].power;
			currentTime = S.oneSecTimer;
		}

		//--------sending BT info--------
		// 500ms timer.
		if ((S.BT_sendState == 1) && (S.BT_transmission_over == 1)){
			if (S.runMode != RUN_PAUSED){
				BTpacketSize = BT_MC_generateStatusMsg(BT_RUN);
			}else{
				BTpacketSize = BT_MC_generateStatusMsg(BT_PAUSE);
			}
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,BTpacketSize);
			S.BT_transmission_over = 0;
			S.BT_sendState = 0;
		}


		if (S.TD_POT_check == 1){ //500ms
			TD_readADC(&tdp);
			if (u.delivery_mMin != tdp.usedDeliveryM_min){
				TD_calculateMaxDraft(&tdp,&u);
				tdp.appliedLevel = 0; // force recalculation of td value when mtr/min changes
			}
			TD_calculate(&tdp);
			if (tdp.tensionDraftChanged ==1){
				C.tensionDraft = tdp.tensionDraft;
				if ((S.runMode == RUN_CARDING_SECTION)&&(S.piecingMode == 0)){
					updateCoilerParameters(&C, &u);
					uint8_t motors[] = {COILER};
					uint16_t targets[] = {C.M.coilerMotorRPM};
					noOfMotors = 1;
					SendChangeTargetToMultipleMotors(motors,noOfMotors,targets);
				}
				tdp.tensionDraftChanged  = 0;
				//enable Beep Logic
				tdp.beepEnable = 1;
				tdp.beepCounter = 0;
				TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_ON,SAME_STATE,SAME_STATE,SAME_STATE);
				TowerLamp_ApplyState(&hmcp,&mcp_portB);
			}

			if (tdp.beepEnable){
				tdp.beepCounter ++;
				if (tdp.beepCounter >2){
					tdp.beepEnable = 0;
					TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,SAME_STATE,SAME_STATE,SAME_STATE);
					TowerLamp_ApplyState(&hmcp,&mcp_portB);
				}
			}
			S.TD_POT_check = 0;
		}

		// stop btn
		if (usrBtns.redBtn == BTN_PRESSED){
			usrBtns.redBtn = BTN_IDLE;
			//STOP
			uint8_t motors[] = {CARDING_CYLINDER,BEATER_CYLINDER,CARDING_FEED,BEATER_FEED,CAGE,COILER,AF_PICKER_CYLINDER,AF_FEED};
			noOfMotors = 8;
			response = SendCommands_To_MultipleMotors(motors,noOfMotors,EMERGENCY_STOP);
			S.runMode = RUN_STOPPED ;
			S.BT_pauseReason = 0;

			//beep once when we go to idle
			TowerLamp_SetState(&hmcp,&mcp_portB,BUZZER_ON,RED_OFF,GREEN_OFF,AMBER_OFF);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);
			HAL_Delay(1000); // to hear the beep

			ChangeState(&S,IDLE_STATE);
			break;
		}

	}//closes while

}


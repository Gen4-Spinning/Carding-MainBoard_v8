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

void RunState(void){

	uint8_t response = 0;
	uint8_t noOfMotors = 0;
	//uint8_t BTpacketSize = 0;
	long currentTime;
	while(1){

		if (S.oneTime){
			//send the start commands
			uint8_t motors[] = {CARDING_CYLINDER,BEATER_CYLINDER,AF_PICKER_CYLINDER};
			noOfMotors = 3;
			response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);

			S.runMode = RUN_RAMPUP;

			//when u start your in the run mode
			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_ON,AMBER_OFF);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);

			S.TD_POT_check = 0; // make this zero when u come into Run state
			S.oneTime = 0;
		}

		/*---------------- Beater Feed and AF Feed Logic -----------------*/
		ductCardFeedTop.currentReading = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_TOP_CARDFEED);
		ductCardFeedTop.ductStateChanged = SensorAppyHysteresis(&ductCardFeedTop);

		ductCardFeedBtm.currentReading = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_BTM_CARDFEED);
		ductCardFeedBtm.ductStateChanged = SensorAppyHysteresis(&ductCardFeedBtm);

		if ((ductCardFeedTop.ductStateChanged) || (ductCardFeedBtm.ductStateChanged)){
			C.D.cardFeedTop_sensorState = ductCardFeedTop.presentState; // putting the sensor states in the C struct
			C.D.cardFeedBtm_sensorState  = ductCardFeedBtm.presentState;
			processCardFeedDuctLevel(&C);
			if ((S.runMode == RUN_FILL_DUCT) || (S.runMode == RUN_CARDING_SECTION)){
				sendCommandToBeaterFeedMotor(&C);
			}
			ductCardFeedTop.ductStateChanged = 0;
			ductCardFeedBtm.ductStateChanged = 0;
		}

		// check duct auto Feed Status. Status is set here, and used in the Run State blocks of code below.
		ductAutoFeed.currentReading = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_AF);
		ductAutoFeed.ductStateChanged = SensorAppyHysteresis(&ductAutoFeed);
		if (ductAutoFeed.ductStateChanged){
			uint8_t autoFeedCommand = 0;
			if ((S.runMode == RUN_FILL_DUCT) || (S.runMode == RUN_CARDING_SECTION)){
				if (C.D.autoFeed_sensorState == DUCT_SENSOR_OPEN){
					C.D.autoFeed_ductState_current = DUCT_OPEN;
					autoFeedCommand = START;
				}else if (C.D.autoFeed_sensorState == DUCT_SENSOR_CLOSED){
					C.D.autoFeed_ductState_current = DUCT_CLOSED;
					autoFeedCommand = RAMPDOWN_STOP;
				}
				sendStartStopToAutoFeedMotor(&C,autoFeedCommand);
			}
			ductAutoFeed.ductStateChanged = 0;
		}

		/*----------------Ending Beater Feed and AF Feed Logic -----------------*/

		//--------run through the various Modes-----------------------------

		if (S.runMode == RUN_RAMPUP){
			C.L.stateMc_rampOver = CheckCylindersRampUpOver(&C,&ER[CARDING_CYLINDER],&ER[BEATER_CYLINDER],&R[AF_PICKER_CYLINDER]);
			if (C.L.stateMc_rampOver == 1){
				S.runMode = RUN_FILL_DUCT;
				//force the cardFeed section above to check the duct state by setting present state to not what it currently is
				ductCardFeedTop.presentState = DUCT_SENSOR_RESET;
				ductCardFeedBtm.presentState = DUCT_SENSOR_RESET;
			}
		}

		if (S.runMode == RUN_FILL_DUCT){
			if (C.D.cardFeed_ductLevel == DUCT_LEVEL_HIGH){
				updateCardingSectionSpeeds(&C,&u);
				S.runMode = RUN_CARDING_SECTION;
				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				noOfMotors = 3;
				response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
			}
		}


		if (S.runMode == RUN_CARDING_SECTION){

			if (updateSettings){
				updateCardingSectionSpeeds(&C,&u);
				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				uint16_t targets[] = {C.M.cardFeedMotorRPM,C.M.cageMotorRPM,C.M.coilerMotorRPM};
				noOfMotors = 3;
				SendChangeTargetToMultipleMotors(motors,noOfMotors,targets);
				updateSettings = 0;
			}

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
				uint8_t motors[] = {CARDING_FEED,CAGE,COILER};
				noOfMotors =3;
				response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);

				S.runMode = RUN_CARDING_SECTION;

				TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_ON,AMBER_OFF);
				TowerLamp_ApplyState(&hmcp,&mcp_portB);
			}
		}


		if (S.oneSecTimer != currentTime){
			C.L.mcPower = ER[0].power + ER[1].power + R[2].power + R[3].power + R[4].power + R[5].power + R[6].power+ R[7].power;
			currentTime = S.oneSecTimer;
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


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

		//check Duct Sensor Status
		/*sensor.ductSensor = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCT_SENSOR);
		sensor.ductStateChanged = SensorAppyHysteresis(&sensor,sensor.ductSensor);
		if (sensor.ductStateChanged == 1){
			sensor.ductCurrentState = sensor.ductSensor;
			sensor.ductTimerIncrementBool = 0;
			sensor.ductSensorTimer = 0;
			sensor.ductStateChanged = 0;
			//duct signals arent allowed during rampup, pause or stop
			if (sensor.ductCurrentState == DUCT_SENSOR_OPEN){
				S.btrFeed_Duct_signal = START_BTR_FEED;
			}
			else if (sensor.ductCurrentState == DUCT_SENSOR_CLOSED){
				S.btrFeed_Duct_signal = STOP_BTR_FEED;
			}else{}
			L.logRunStateChange = 1;
		}

		//the if-else if is important here cos it ensures the flow first checks the start btr feed and only
		// if its not there does the duct signalling.
		if (S.runMode == RUN_ALL){
			if (S.btrFeed_nonDuct_signal == START_BTR_FEED){
				if (sensor.ductCurrentState== DUCT_SENSOR_OPEN){
					//will send whatevers been setup/piecing or non piecing.
					uint8_t motors[] = {BEATER_FEED};
					noOfMotors = 1;
					//TODO something with response
					response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
					S.btrFeed_nonDuct_signal = NOTHING_BUFFERED_BTR_FEED;
					S.btrFeed_Duct_signal = NOTHING_BUFFERED_BTR_FEED;
				}
			}
			else{
				if (S.btrFeed_Duct_signal == START_BTR_FEED){
					uint8_t motors[] = {BEATER_FEED};
					noOfMotors = 1;
					//TODO something with response
					response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
					S.btrFeed_nonDuct_signal = NOTHING_BUFFERED_BTR_FEED;
					S.btrFeed_Duct_signal = NOTHING_BUFFERED_BTR_FEED;

				}
				else if (S.btrFeed_Duct_signal == STOP_BTR_FEED){
					uint8_t motors[] = {BEATER_FEED};
					noOfMotors = 1;
					//TODO something with response
					response = SendCommands_To_MultipleMotors(motors,noOfMotors,RAMPDOWN_STOP);
					S.btrFeed_nonDuct_signal = NOTHING_BUFFERED_BTR_FEED;
					S.btrFeed_Duct_signal = NOTHING_BUFFERED_BTR_FEED;
				}
				else{}
			}
		}
		 */

		if (updateSettings){
			updateMachineSpeeds(&C,&u,u.delivery_mMin,0);
			uint8_t motors[] = {CARDING_FEED,BEATER_FEED,CAGE,COILER,AF_FEED};
			uint16_t targets[] = {C.M.cardFeedMotorRPM,C.M.btrFeedMotorRPM,
							C.M.cageMotorRPM,C.M.coilerMotorRPM,C.M.afFeedMotorRPM};
			noOfMotors = 5;
			SendChangeTargetToMultipleMotors(motors,noOfMotors,targets);
			updateSettings = 0;
		}

		if (S.runMode == RUN_RAMPUP){
			C.L.stateMc_rampOver = CheckCylindersRampUpOver(&C,&ER[CARDING_CYLINDER],&ER[BEATER_CYLINDER],&R[AF_PICKER_CYLINDER]);
			if (C.L.stateMc_rampOver == 1){
				S.runMode = RUN_FILL_DUCT;

				updateMachineSpeeds(&C,&u,I.ductFillDeliveryMtrsMin,0);
				ReadySetupCommand_AllMotors(&C);

				uint8_t motors[] = {BEATER_FEED,AF_FEED};
				noOfMotors = 2;
				response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
				// Turn on Btr Feed based on Duct State
				S.btrFeed_nonDuct_signal = START_BTR_FEED;
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


/*
 * IdleState.c
 *
 *  Created on: 15-Apr-2023
 *      Author: harsha
 *
 */

#include "stdio.h"
#include "StateMachine.h"
#include "CommonConstants.h"
#include "CAN_MotherBoard.h"
#include "MotorComms.h"
#include "Ack.h"
#include "userButtons.h"
#include "FDCAN.h"
#include "SMPS.h"

#include "mcp23017.h"
#include "AC_SSR.h"
#include "TD_Pot.h"

#include "MachineErrors.h"
#include "BT_Fns.h"
#include "BT_Machine.h"
#include "BT_Console.h"

#include "TowerLamp.h"
#include "DataRequest.h"

uint8_t doActivity = 0 ;

uint8_t dbg_Start = 0;
uint8_t dbg_stop = 0;
uint8_t dbg_pause = 0;
uint8_t dbg_resume = 0;

uint8_t response = 0;
uint8_t stopSMPSNow = 0;

uint8_t motors[] = {CARDING_FEED,BEATER_FEED,CAGE};
uint8_t noOfMotors = 3;
uint8_t sendBTCmd,btReturn,btCmd = 0;

uint8_t BT_errDbg = 0;
uint8_t BT_pauseDbg = 0;
uint8_t BT_runDbg = 0;
uint8_t BTpacketSize = 0;
uint16_t errSource1 = 0 ;

uint8_t testTowerLamp = 0;
uint8_t SMPS_on,SMPS_off;
uint8_t canTest = 0;

uint8_t AC_SSR_off;
uint8_t AC_SSR_on;
uint8_t checkAC_SSR;
uint8_t AC_SSR_state;

uint8_t check_TD_pot = 0;
uint8_t TD_counter = 0;

uint8_t requestPID = 0 ;
uint8_t updatePID = 0;
uint8_t PIDmotorNo = 0;
uint8_t PID_DBG_MSG = 0;
uint8_t PIDchk = 0;

uint8_t updateSettingsDbg = 0;
uint8_t cardTop=0,cardBtm=0,feedSensor = 0;

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc2;
extern DataReq DR;

void DebugState(void){

	while(1){
		if (AC_SSR_off){
			setSSR_State(AC_SSR_OFF,&hmcp, &mcp_portA);
			AC_SSR_off = 0;
		}
		if (AC_SSR_on){
			setSSR_State(AC_SSR_ON,&hmcp, &mcp_portA);
			AC_SSR_on = 0;
		}
		if (checkAC_SSR){
			if (CheckAC_SSR(AC_SSR_OFF,&hmcp, &mcp_portA)){ // will return true if SSR is off
				AC_SSR_state=0;
			}else{
				AC_SSR_state=1;
			}
			checkAC_SSR = 0;
		}

		if (check_TD_pot){
			TD_readADC(&tdp);
			if (msp.delivery_mMin != tdp.usedDeliveryM_min){
				TD_calculateMaxDraft(&tdp,&u);
				tdp.appliedLevel = 0; // force recalculation of td value when mtr/min changes
			}
			TD_calculate(&tdp);
			if (tdp.tensionDraftChanged ==1){
				TD_counter++;
				tdp.tensionDraftChanged = 0;
			}
			HAL_Delay(100);
		}

		if (SMPS_on){
			SMPS_TurnOn();
			SMPS_on = 0;
		}
		if (SMPS_off){
			SMPS_TurnOff();
			SMPS_off = 0;
		}

		if(dbg_Start){
			response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
			dbg_Start = 0;
		}

		if (dbg_pause){
			response = SendCommands_To_MultipleMotors(motors,noOfMotors,RAMPDOWN_STOP);
			dbg_pause =0;
		}

		if (dbg_resume){
			response = SendCommands_To_MultipleMotors(motors,noOfMotors,RESUME);
			dbg_resume = 0;
		}

		if (dbg_stop){
			response = SendCommands_To_MultipleMotors(motors,noOfMotors,EMERGENCY_STOP);
			dbg_stop = 0;
		}


		if (canTest){
			SendCommands_To_MultipleMotors(motors,noOfMotors,START);
			HAL_Delay(1000);
			canTest = 0;
		}

		//BT command mode commands
		if (sendBTCmd){
			btReturn = BTConsole_WriteCMD(btCmd);
			sendBTCmd = 0;
		}

		if (BT_errDbg){
			if (ME.errType1 == ERR_MOTOR_SOURCE){
				errSource1 = GetBTMotorID_from_Motor_ID(ME.errSource1);
			}else{
				errSource1 = ME.errSource1;
			}
			SetBTErrors(&ME,ME.errReason1,errSource1,ME.errCode1);
			BTpacketSize = BT_MC_generateStatusMsg(BT_STOP);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,BTpacketSize);
			BT_errDbg = 0;
		}

		if (BT_pauseDbg){
			BTpacketSize = BT_MC_generateStatusMsg(BT_PAUSE);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,BTpacketSize);
			BT_pauseDbg = 0;
		}

		if ((S.BT_sendState == 1) && (S.BT_transmission_over == 1)){
			BT_runDbg = 1;
		}
		if (BT_runDbg){
			BTpacketSize = BT_MC_generateStatusMsg(BT_RUN);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,BTpacketSize);
			BT_runDbg = 0;
			S.BT_transmission_over = 0;
			S.BT_sendState = 0;
		}

		if (S.switchState == TO_SETTINGS){
			ChangeState(&S,SETTINGS_STATE);
			S.switchState = 0;
			break;
		}

		if (updateSettingsDbg){
			updateMachineSpeeds(&C,&u,u.delivery_mMin,0);
			updateSettingsDbg =0;
		}


		//Check duct Sensors:
		cardTop = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_TOP_CARDFEED);
		cardBtm = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_BTM_CARDFEED);
		feedSensor = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCTSENSOR_AF);




		if (testTowerLamp){
			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_OFF,AMBER_ON);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);
			HAL_Delay(5000);

			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_ON,AMBER_OFF);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);
			HAL_Delay(5000);

			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_ON,GREEN_OFF,AMBER_OFF);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);
			HAL_Delay(5000);

			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_ON,RED_ON,GREEN_ON,AMBER_ON);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);
			HAL_Delay(5000);

			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_OFF,AMBER_OFF);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);
			HAL_Delay(5000);
		}


		if (PIDchk==1){
			if (requestPID){
				SendDataRequest(&DR,PID_SETTINGS_REQUEST,PIDmotorNo);
			}else if(updatePID){
				SendDataRequest(&DR,PID_SETTINGS_UPDATE,PIDmotorNo);
			}

			while((DR.timer < DATA_REQUEST_TIMEOUT_SEC) && (DR.responseRecieved == 0)){
				if (DR.responseRecieved == 0){
					// no response error!
					PID_DBG_MSG = 0;
				}else{
					//parse responsed by CAN
					PID_DBG_MSG = 1;
				}
				DR.requestSent = 0;
				DR.requestType = 0;
				DR.motorName = 0;
				DR.timer = 0;
			}
			requestPID = 0;
			updatePID = 0;
			PIDchk = 0;
		}




	}//closes while

}


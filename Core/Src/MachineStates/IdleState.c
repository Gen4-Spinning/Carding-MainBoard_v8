/*
 * IdleState.c
 *
 *  Created on: 15-Apr-2023
 *      Author: harsha
 *
 *  Things to Do in Idle State for Flyer:
 *  1. Detect Button Press and do homing/ inching and start the full run cycle
 *  2. Save New settings from the app when we get them
 *  3. Write the Dbg codes to start motors. Dbg Stop is in the while loop.
 */

#include "stdio.h"
#include "MachineErrors.h"
#include "StateMachine.h"
#include "CommonConstants.h"
#include "CAN_MotherBoard.h"
#include "MotorComms.h"
#include "Ack.h"
#include "userButtons.h"
#include "BT_Fns.h"
#include "TowerLamp.h"
#include "mcp23017.h"
#include "MachineSensors.h"
#include "Log.h"
#include "TD_Pot.h"

extern UART_HandleTypeDef huart1;
extern uint8_t changeBtrFeedState;
void IdleState(void){

	/* The rotary switch enables or disable the coiler sensor
	 * green button starts the run
	 * yellow does nothing
	 * red does nothing
	 *
	 * also from this state only, you can go to the diagnostics or change the settings
	 * and save. Only for Carding you can change even in run mode
	 *
	 * When u press the red Btn in runState you come back to idle. At that point we want to clear
	 * any runTime vars so that user is force to start a new run.
	 */
	uint8_t response = 0;
	uint8_t BTpacketSize = 0;
	while(1){

		if (S.oneTime){
			TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_OFF,AMBER_ON);
			TowerLamp_ApplyState(&hmcp,&mcp_portB);

			setupCardingMCType(&C,&u);
			ReadySetupCommand_AllMotors(&C);
			//Reset Run machine Parameters
			C.L.currentMtrsRun = 0.01;
			C.L.mcPower = 0.01;

			//L.logRunStateChange = 1;
			//L.flushBuffer = 1;
			S.oneTime  = 0;
		}

		if (usrBtns.greenBtn == BTN_PRESSED){
			usrBtns.greenBtn = BTN_IDLE;
			//Log_ResetRunTimeRdngNos();
			ChangeState(&S,RUN_STATE);
			break;
		}

		//----------- go to other places-------

		if (S.switchState == TO_SETTINGS){
			ChangeState(&S,SETTINGS_STATE);
			S.switchState = 0;
			break;
		}else if (S.switchState == TO_DIAGNOSTICS){
			ChangeState(&S,DIAGNOSTICS_STATE);
			S.switchState = 0;
			break;
		}
		//--------ways to go into Error State--------

		if (response == 2){
			//Setup command Error. No motor would have started. just
			//Send a snackbar to the app
		}

		//Error State
		if(ME.ErrorFlag == 1){
			ChangeState(&S,ERROR_STATE);
			break;
		}

		//--------sending BT info--------
		// 500ms timer.
		if ((S.BT_sendState == 1) && (S.BT_transmission_over == 1)){
				BTpacketSize = BT_MC_generateStatusMsg(BT_IDLE);
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,BTpacketSize);
			S.BT_transmission_over = 0;
			S.BT_sendState = 0;
		}

		//in idle only the state change gets logged. no motor running
		/*if (S.LOG_enabled){
			Log_DoOneCycle();
		}*/

		//update the draft in IDLE also, can be used to see the limits also
		//by changing the delivery mts min from the app.
		if (S.TD_POT_check == 1){ //500ms
			TD_readADC(&tdp);
			if (u.delivery_mMin != tdp.usedDeliveryM_min){
				TD_calculateMaxDraft(&tdp,&u);
				tdp.appliedLevel = 0; // force recalculation of td value when mtr/min changes
			}
			TD_calculate(&tdp);
			if (tdp.tensionDraftChanged ==1){
				msp.draft = tdp.tensionDraft;
				tdp.tensionDraftChanged  = 0;

				//enable Beep Logic
				tdp.beepEnable = 1;
				tdp.beepCounter = 0;
				TowerLamp_NegateState(&hmcp, &mcp_portB,TOWER_BUZZER);
				TowerLamp_ApplyState(&hmcp,&mcp_portB);
			}

			if (tdp.beepEnable){
				tdp.beepCounter ++;
				if (tdp.beepCounter >2){
					tdp.beepEnable = 0;
					TowerLamp_NegateState(&hmcp, &mcp_portB,TOWER_BUZZER);
					TowerLamp_ApplyState(&hmcp,&mcp_portB);
				}
			}
			S.TD_POT_check = 0;
		}

		//-------for Manual change--------
		if (S.current_state != IDLE_STATE){
			break;
		}


	}//closes while

}


/*
 * IntDriveComms.c
 *
 *  Created on: 17-Apr-2023
 *      Author: harsha
 */

#include "MotorComms.h"
#include "stdio.h"
#include "Struct.h"
#include "CommonConstants.h"
#include "CAN_MotherBoard.h"
#include "Ack.h"

void ReadySetupRPMCommand_CardingMotors(CardingMc *C){
	SU[CARDING_CYLINDER].RPM = C->M.cardCylMotorRPM;
	SU[BEATER_CYLINDER].RPM = C->M.btrCylMotorRPM;
	SU[CAGE].RPM = C->M.cageMotorRPM;
	SU[CARDING_FEED].RPM = C->M.cardFeedMotorRPM;
	SU[COILER].RPM = C->M.coilerMotorRPM;
}

void ReadySetupCommand_AllMotors(CardingMc *C){
	SU[CARDING_CYLINDER].RPM = C->M.cardCylMotorRPM;
	SU[BEATER_CYLINDER].RPM = C->M.btrCylMotorRPM;
	SU[CAGE].RPM = C->M.cageMotorRPM;
	SU[CARDING_FEED].RPM = C->M.cardFeedMotorRPM;
	SU[BEATER_FEED].RPM = C->M.btrFeedMotorRPM;
	SU[COILER].RPM = C->M.coilerMotorRPM;
	SU[AF_PICKER_CYLINDER].RPM = C->M.pickerCylMotorRPM;
	SU[AF_FEED].RPM = C->M.afFeedMotorRPM;
	for (int i= 0;i<2;i++){
		SU[i].RDT = CYLINDERS_RAMP_TIME_SEC;
		SU[i].RUT = CYLINDERS_RAMP_TIME_SEC;
	}
	for (int i=2;i<7;i++){
		SU[i].RDT = NON_CYLINDER_RAMPDOWN_TIME_SEC;
		SU[i].RUT = NON_CYLINDER_RAMPUP_TIME_SEC;
	}
	SU[AF_FEED].RDT = AF_FEED_RAMPDOWN_TIME_SEC;
	SU[AF_FEED].RUT = AF_FEED_RAMPUP_TIME_SEC ;

	//picker Cylinder
	SU[AF_PICKER_CYLINDER].RDT = CYLINDERS_RAMP_TIME_SEC;
	SU[AF_PICKER_CYLINDER].RUT = CYLINDERS_RAMP_TIME_SEC;

}

uint8_t  SendCommands_To_MultipleMotors(uint8_t *motorList,uint8_t motorArraySize,uint8_t command){
	/* Returns 0 if the command has ran into Ack errors and we have finally sent a STOP command/ or we ve just send a stop Command.
	 * Returns 1 if the command has run properly and we ve recieved the correct no of Acks for Setup/start/RD and RU
	 * Return 2 is the Setup Command has not recieved the right no of ACKS
	 *
	 * The CAN timer is a 15ms interrupt which is very large! when compared to the time
	 * needed to actually get an ack, so if we havent recieved its quite certain theres
	 * some fault. The CAN interrupt for the ack msg handles the success case by stopping
	 * the timer if all the acks are recieved.
	 */

	uint8_t noOfMotors = 0;
	uint16_t motorAcksCheck = 0;
	uint8_t canID;
	uint8_t motor;
	uint8_t motorAddresses[6]={};

	noOfMotors = motorArraySize;

	for (int i=0;i<noOfMotors;i++){
		canID = getMotorCANAddress(motorList[i]);
		motorAddresses[i] = canID;
		motorAcksCheck |= (1<< (canID-2));
	}

	if (command == START){
		reset_ACKs();
		//say ack for what - so that if it fails we know what it was looking for when it failed.
		ACK_startCheck(motorAcksCheck,ACK_FOR_SETUP_MM,NON_CRITICAL_ACK); // StartMM - MM is for multipleMotors
		for (int i=0;i<noOfMotors;i++){
			canID = motorAddresses[i];
			motor = motorList[i];
			FDCAN_sendSetUp_ToMotor(canID,SU[motor]);
			HAL_Delay(1); // is needed here or the second msg doesnt seem to go!
		}

		while(ack.waitingForAckResult){};

		if (ack.ackResult == ACK_SUCCESS){
			reset_ACKs();
			ACK_startCheck(motorAcksCheck,ACK_FOR_START_MM, NON_CRITICAL_ACK);
			for (int i=0;i<noOfMotors;i++){
				canID = motorAddresses[i];
				FDCAN_sendCommand_ToMotor(canID, START);
			}
			while(ack.waitingForAckResult){};

			if (ack.ackResult ==  ACK_FAIL){
				reset_ACKs();
				ACK_startCheck(motorAcksCheck,ACK_FOR_START_MM,CRITICAL_ACK);
				for (int i=0;i<noOfMotors;i++){
					canID = motorAddresses[i];
					FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
				}
				return 0;
			}
			return 1;
		}
		else{	// Didnt recieve Ack for Setup
			return 2;
			}

	} // closes START

	if (command == RAMPDOWN_STOP){
			reset_ACKs();
			ACK_startCheck(motorAcksCheck,ACK_FOR_RD_MM, NON_CRITICAL_ACK);
			for (int i=0;i<noOfMotors;i++){
				canID = motorAddresses[i];
				FDCAN_sendCommand_ToMotor(canID, RAMPDOWN_STOP);
			}
			while(ack.waitingForAckResult){};

			if (ack.ackResult ==  ACK_FAIL){
				reset_ACKs();
				ACK_startCheck(motorAcksCheck,ACK_FOR_RD_MM,CRITICAL_ACK);
				for (int i=0;i<noOfMotors;i++){
					canID = motorAddresses[i];
					FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
				}
				return 0;
			}
			return 1;
		}

		if (command == RESUME){
			reset_ACKs();
			ACK_startCheck(motorAcksCheck,ACK_FOR_RU_MM, NON_CRITICAL_ACK);
			for (int i=0;i<noOfMotors;i++){
				canID = motorAddresses[i];
				FDCAN_sendCommand_ToMotor(canID, RESUME);
			}
			while(ack.waitingForAckResult){};
			if (ack.ackResult ==  ACK_FAIL){
				reset_ACKs();
				ACK_startCheck(motorAcksCheck,ACK_FOR_RU_MM,CRITICAL_ACK);
				for (int i=0;i<noOfMotors;i++){
					canID = motorAddresses[i];
					FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
				}
				return 0;
			}
			return 1;
		}

		if (command == HOMING){
			reset_ACKs();
			ACK_startCheck(motorAcksCheck,ACK_FOR_HOMING_MM, NON_CRITICAL_ACK);
			for (int i=0;i<noOfMotors;i++){
				canID = motorAddresses[i];
				FDCAN_sendCommand_ToMotor(canID, HOMING);
			}
			while(ack.waitingForAckResult){};
			if (ack.ackResult ==  ACK_FAIL){
				reset_ACKs();
				ACK_startCheck(motorAcksCheck,ACK_FOR_HOMING_MM,CRITICAL_ACK);
				for (int i=0;i<noOfMotors;i++){
					canID = motorAddresses[i];;
					FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
				}
				return 0;
			}
			return 1;
		}

		if (command == EMERGENCY_STOP){
			reset_ACKs();
			ACK_startCheck(motorAcksCheck,ACK_FOR_STOP_MM,CRITICAL_ACK);
			for (int i=0;i<noOfMotors;i++){
				canID = motorAddresses[i];
				FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
			}
			return 0;
		}

	//should never reach here
	return 0;
}

uint8_t  Send_DiagCommands_To_MultipleMotors(uint8_t *motorList,uint8_t motorArraySize,uint8_t command){
	/* Returns 0 if the command has ran into Ack errors and we have finally sent a STOP command/ or we ve just send a stop Command.
	 * Returns 1 if the command has run properly and we ve recieved the correct no of Acks for Setup/start/RD and RU
	 * Return 2 is the Setup Command has not recieved the right no of ACKS
	 */

	uint8_t noOfMotors = 0;
	uint16_t motorAcksCheck = 0;
	uint8_t canID;
	uint8_t motorAddresses[6]={};

	noOfMotors = motorArraySize;

	for (int i=0;i<noOfMotors;i++){
		canID = getMotorCANAddress(motorList[i]);
		motorAddresses[i] = canID;
		motorAcksCheck |= (1<< (canID-2));
	}

	if (command == START){
		reset_ACKs();
		//say ack for what - so that if it fails we know what it was looking for when it failed.
		ACK_startCheck(motorAcksCheck,ACK_FOR_DIAG_SETUP,NON_CRITICAL_ACK);
		for (int i=0;i<noOfMotors;i++){
			canID = motorAddresses[i];
			FDCAN_SendDiagnostics_ToMotor(canID,&D);
			HAL_Delay(1); // is needed here or the second msg doesnt seem to go!
		}


		while(ack.waitingForAckResult){};

		if (ack.ackResult == ACK_SUCCESS){
			reset_ACKs();
			ACK_startCheck(motorAcksCheck,ACK_FOR_DIAG_START, NON_CRITICAL_ACK);
			for (int i=0;i<noOfMotors;i++){
				canID = motorAddresses[i];
				FDCAN_sendCommand_ToMotor(canID, START);
			}
			while(ack.waitingForAckResult){};

			if (ack.ackResult ==  ACK_FAIL){
				//TODO detect which motor Failed.
				reset_ACKs();
				ACK_startCheck(motorAcksCheck,ACK_FOR_DIAG_START,CRITICAL_ACK);
				for (int i=0;i<noOfMotors;i++){
					canID = motorAddresses[i];
					FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
				}
				return 0;
			}
			return 1;
		}
		else{	// Didnt recieve Ack for Setup
			return 2;
			}

	} // closes START

	if (command == EMERGENCY_STOP){
		reset_ACKs();
		ACK_startCheck(motorAcksCheck,ACK_FOR_DIAG_STOP,CRITICAL_ACK);
		for (int i=0;i<noOfMotors;i++){
			canID = motorAddresses[i];
			FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
		}
		return 0;
	}

	//should never reach here
	return 0;
}


uint8_t SendChangeTargetToMultipleMotors(uint8_t *motorList,uint8_t motorArraySize,uint16_t *changeTargets){
	uint8_t noOfMotors = 0;
	uint16_t targetRpm = 0;
	uint16_t motorAcksCheck = 0;
	uint8_t canID;
	uint8_t motorAddresses[6]={};

	noOfMotors = motorArraySize;

	for (int i=0;i<noOfMotors;i++){
		canID = getMotorCANAddress(motorList[i]);
		motorAddresses[i] = canID;
		motorAcksCheck |= (1<< (canID-2));
	}

	reset_ACKs();
	ACK_startCheck(motorAcksCheck,ACK_FOR_CHANGERPM, NON_CRITICAL_ACK);
	for (int i=0;i<noOfMotors;i++){
		canID = motorAddresses[i];
		targetRpm = changeTargets[i];
		FDCAN_sendChangeTarget_ToMotor(canID,targetRpm,I.changeRPM_rampTimes*1000);
	}
	while(ack.waitingForAckResult){};
	if (ack.ackResult ==  ACK_FAIL){
		reset_ACKs();
		ACK_startCheck(motorAcksCheck,ACK_FOR_CHANGERPM,CRITICAL_ACK);
		for (int i=0;i<noOfMotors;i++){
			canID = motorAddresses[i];;
			FDCAN_sendCommand_ToMotor(canID, EMERGENCY_STOP);
		}
		return 0;
	}

	return 1;
}


uint8_t sendStartStopToAutoFeedMotor(CardingMc *c,uint8_t state){
	uint8_t motors[] = {AF_FEED};
	uint8_t noOfMotors = 1;
	uint8_t response = 0;
	if (state == START){
		response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
	}else if (state == RAMPDOWN_STOP){
		response = SendCommands_To_MultipleMotors(motors,noOfMotors,RAMPDOWN_STOP);
	}
	return response;
}

uint8_t sendStartStopToBeaterFeedMotor(CardingMc *c,uint8_t state){
	uint8_t motors[] = {BEATER_FEED};
	uint8_t noOfMotors = 1;
	uint8_t response = 0;
	if (state == START){
		response = SendCommands_To_MultipleMotors(motors,noOfMotors,START);
	}else if (state == RAMPDOWN_STOP){
		response = SendCommands_To_MultipleMotors(motors,noOfMotors,RAMPDOWN_STOP);
	}
	return response;
}

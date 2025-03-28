/*
 * PID_Settings.c
 *
 *  Created on: 22-Aug-2024
 *      Author: harsha
 */

#include "../DataRequest/DataRequest.h"

void SendDataRequest(DataReq *d,uint8_t requestType,uint8_t motorName){
	uint8_t motorCANID;
	d->requestType = requestType;
	d->motorName = motorName;
	if (d->requestType == PID_SETTINGS_REQUEST){
		motorCANID = getMotorCANAddress(d->motorName);
		FDCAN_sendDataRequest_ToMotor(motorCANID,PID_SETTINGS_REQUEST);
		d->requestSent = 1;
	}
	else if(d->requestType == PID_SETTINGS_UPDATE){
		motorCANID = getMotorCANAddress(d->motorName);
		FDCAN_sendPID_Update_toMotor(motorCANID);
		d->requestSent = 1;
	}

}

/*
 * PID_Settings.h
 *
 *  Created on: 22-Aug-2024
 *      Author: harsha
 */

#ifndef DATAREQUEST_H_
#define DATAREQUEST_H_

#include "machineSettings.h"
#include "CAN_MotherBoard.h"

//Data Request Types
#define PID_SETTINGS_REQUEST 1
#define PID_SETTINGS_UPDATE 2


#define DATA_REQUEST_TIMEOUT_SEC 2 // 2 sec request timeout

typedef struct PIDRequest{
	uint8_t motorID;

	float Kp_motorID;
	float Ki_motorID;
	float FF_motorID;
	uint16_t startOffset_motorID;

	float newKp;
	float newKi;
	float newFF;
	uint16_t newStartOffset;
}PIDReq;


typedef struct DataRequestTD{
	uint8_t requestType;
	uint8_t motorName;
	PIDReq p;
	char requestSent;
	char responseRecieved;
	uint8_t timer;
}DataReq;


extern DataReq DR;

void SendDataRequest(DataReq *d,uint8_t requestType,uint8_t motorName);


#endif /* DATAREQUEST_H_ */

/*
 * Struct.h
 *
 *  Created on: 06-Mar-2023
 *      Author: harsha
 */

#ifndef INC_STRUCT_H_
#define INC_STRUCT_H_

#include "stdio.h"
#include "stdint.h"

typedef struct Diagnostics_Struct
{
	char typeofTest;
	char motorID;
    int targetRPM;
    int targetDuty;
    uint16_t runTime;
    char rampUpTime;
    char rampDownTime;
    char direction;
    uint16_t bedDist;
    char stopTest;
}DiagnosticsTypeDef;


typedef struct MotorRunTimeStruct
{	uint16_t rdngNo;
	float targetPosition;
	float presentPosition;
	uint16_t targetRPM;
	uint16_t presentRPM;
	uint16_t pwm;
	uint8_t mosfetTemp;
	uint8_t motorTemp;
	uint16_t currentRaw;
	uint16_t voltageRaw;
	float currentA;
	float voltageV;
	uint16_t proportionalTerm;
	uint16_t integralTerm;
	uint16_t feedForwardTerm;
	uint16_t motorError;
	float power;
}RunTime_TypeDef;

typedef struct ExtendedMotorRunTimeStruct
{	int16_t actualRPM;
	int16_t targetRPM;
	uint16_t busVoltage;
	int16_t idaxis;
	int16_t iqaxis;
	int16_t peakPhaseCurrent;
	int16_t peakPhaseVoltage;
	float peakPhaseCurrentApk;
	float peakPhaseVoltageVpk;
	uint8_t FETTemp;
	uint8_t MOTTemp;
	uint16_t prevFault;
	uint16_t currFault;
	int16_t power;
	uint8_t motorState;
}ExtendedRunTime_TypeDef;

extern DiagnosticsTypeDef D;
extern RunTime_TypeDef R[8];
extern ExtendedRunTime_TypeDef ER[2];


void InitializeDiagnostic_TypeDef(DiagnosticsTypeDef *d);
void InitializeRunTime_TypeDef(void);

#endif /* INC_STRUCT_H_ */

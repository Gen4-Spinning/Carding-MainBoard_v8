/*
 * IntDriveComms.h
 *
 *  Created on: 17-Apr-2023
 *      Author: harsha
 */

#ifndef MOTORCOMMS_H_
#define MOTORCOMMS_H_

#include "machineSettings.h"

typedef struct Setup_NormalMotor_Struct{
	uint16_t RUT;
	uint16_t RDT;
	uint16_t RPM;
}SetupMotor;

extern SetupMotor SU[8];

void ReadySetupCommand_AllMotors(CardingMc *C);

uint8_t SendCommands_To_MultipleMotors(uint8_t *motorList,uint8_t motorArraySize,uint8_t command);
uint8_t Send_DiagCommands_To_MultipleMotors(uint8_t *motorList,uint8_t motorArraySize,uint8_t command);
uint8_t SendChangeTargetToMultipleMotors(uint8_t *motorList,uint8_t motorArraySize,uint16_t *changeTargets);

uint16_t updateBeaterFeedMotorRPM_BasedOnDuctLevel(CardingMc *c);
uint8_t sendCommandToBeaterFeedMotor(CardingMc *c);
uint8_t sendStartStopToAutoFeedMotor(CardingMc *c,uint8_t state);

#endif /* MOTORCOMMS_H_ */

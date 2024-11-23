/*
 * machineSettings.h
 *
 *  Created on: Apr 11, 2023
 *      Author: harsha
 */

#ifndef MACHINESETTINGS_H_
#define MACHINESETTINGS_H_

#include "stdio.h"
#include "FDCAN.h"
#include "Struct.h"

//all Structs with arrays have to follow this Form
#define CARDING_CYLINDER 0
#define BEATER_CYLINDER 1
#define CAGE 2
#define CARDING_FEED 3
#define BEATER_FEED 4
#define COILER 5
#define AF_PICKER_CYLINDER 6
#define AF_FEED 7

#define CYLINDER_GEAR_RATIO 0.25f
#define BEATER_FEED_GB 120
#define CYLINDER_FEED_GB 120
#define TONGUE_GROOVE_CIRCUMFERENCE_MM 213.63
#define TG_TO_GB_RATIO 1

#define CAGE_CIRCUMFERENCE_MM	468.0973
#define CAGE_TO_GB_RATIO 2.6389
#define CAGE_GB 5

#define COILER_GROOVE_CIRCUMFERENCE_MM 194.779
#define COILER_GROOVE_TO_GB_RATIO 1.656
#define COILER_GB 6.91

#define AF_FEED_GB 180
#define AF_PICKER_CYL_GEAR_RATIO 0.5f

#define CYLINDERS_RAMP_TIME_SEC 60
#define CYL_INVERTERS_S16_TO_AMPS 0.000457764f
#define CYL_INVERTERS_S16_TO_VOLTS 0.00704794f

//feeds,cage,coiler
#define NON_CYLINDER_RAMPUP_TIME_SEC 3
#define NON_CYLINDER_RAMPDOWN_TIME_SEC 2

typedef struct rollerRPMsType{
	float cardFeedRPM;
	float btrFeedRPM;
	float pickerFeedRPM;
	float TgRPM;
	float cageRPM;
	float coilerGBShaftRPM;
}rollerRPMs;

typedef struct motorRPMsType{
	uint16_t cardCylMotorRPM;
	uint16_t btrCylMotorRPM;
	uint16_t pickerCylMotorRPM;
	uint16_t cardFeedMotorRPM;
	uint16_t btrFeedMotorRPM;
	uint16_t afFeedMotorRPM;
	uint16_t cageMotorRPM;
	uint16_t coilerMotorRPM;
}motorRPMS;


typedef struct liveValuesType{

	uint8_t pickerFeedOn;
	uint8_t btrFeedOn;

	uint8_t piecingState;

	uint8_t stateMc_rampOver;
	uint8_t stateMc_ductFillOver;

	float mcPower;
	float currentMtrsRun;
}liveValues;

typedef struct userSettingsType{
	float delivery_mMin;
	float lengthLimit;

	//----advanced-----
	uint16_t cardCylRPM;
	uint16_t btrCylRPM;
	uint16_t pickerCylRPM;
	float btrFeedRPM;
	float AF_FeedRPM;

	float deliveryMtrMin_CardFeed_Ratio;

}userSettings;

typedef struct internalSettingsType{
	uint16_t AF_ductSensorDelay;
	uint16_t cardingDuctSensorTopDelay;
	uint16_t cardingDuctSensorBtmDelay;
	uint16_t piecingDeliveryMtrsMin;
}internalSettings;

typedef struct DuctType{
	uint16_t autoFeed_sensorState;
	uint16_t cardFeedTop_sensorState;
	uint16_t cardFeedBtm_sensorState;
	uint8_t cardFeed_ductLevel;

	uint8_t cardFeed_ductState_current;
	uint8_t autoFeed_ductState_current;
}ducts;

typedef struct cardingMC_Full{
	float cardingDelivery_mtrMin;
	float lengthLimit;
	float tensionDraft;
	uint8_t machineState;

	motorRPMS M;
	rollerRPMs R;
	ducts D;
	liveValues L;

}CardingMc;



typedef struct machineSettings_Struct{
	float delivery_mMin;
	float draft;
	uint16_t cylinderSpeed;
	float cylinderFeed ;
	uint16_t beaterSpeed ;
	float beaterFeed ;
	uint16_t trunkDelay ;
	uint16_t lengthLimit;
    int rampTimes;	// only for non cylinder Motors
}machineSettingsTypeDef;

typedef struct machineParameters_Struct{
    uint16_t cylinderRPM;
    uint16_t beaterRPM;
    uint16_t beaterFeedRPM;
    uint16_t cylinderFeedRPM;
    uint16_t cageRPM;
    uint16_t coilerRPM;

    float cageGB_RPM; // maybe we can measure and see if this is right
    float coilerGB_rpm;

    float allMotorsOn;

	float currentMtrsRun;
	float totalPower;

}machineParamsTypeDef;

extern userSettings u;
extern internalSettings I;
extern CardingMc C;


extern machineSettingsTypeDef msp;
extern machineSettingsTypeDef ps;
extern machineSettingsTypeDef msp_BT;

// Eeprom MachineSettings
void ReadUserSettingsFromEeprom(userSettings *u);
uint8_t WriteUserSettingsIntoEeprom(userSettings *u);
uint8_t CheckUserSettings(userSettings *u);
void LoadDefaultUserSettings(userSettings *u);

//otherFns
void InitInternalSettings(internalSettings *i);
void setupCardingMCType(CardingMc *c,userSettings *u);
uint8_t CheckCylindersRampUpOver(CardingMc *c,ExtendedRunTime_TypeDef *cylinder,ExtendedRunTime_TypeDef *beater,RunTime_TypeDef *pickerCylinder);
void updateCardingSectionSpeeds(CardingMc *c,userSettings *u);
void updateFeedSectionSpeeds(CardingMc *c,userSettings *u);

//coiler and potentiometer related functions
//void updateCoilerParameters(machineSettingsTypeDef *ms,machineParamsTypeDef *m);
uint16_t calcBaseCoilerRPM(userSettings *u);
void updateCoilerParameters(CardingMc *c, userSettings *u);

uint8_t getMotorCANAddress(uint8_t motor);
uint8_t GetMotorID_from_CANAddress(uint8_t canAddress);

#endif /* MACHINESETTINGS_H_ */

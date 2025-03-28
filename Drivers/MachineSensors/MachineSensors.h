/*
 * MachineSensors.h
 *
 *  Created on: May 11, 2023
 *      Author: harsha
 */

#ifndef MACHINESENSORS_H_
#define MACHINESENSORS_H_

#include "mcp23017.h"
#include "machineSettings.h"

#define UNKNOWN_SENSOR 0
#define DUCTSENSOR_TOP_CARDFEED 1
#define TG_OPTICAL_SENSOR 2
#define DUCTSENSOR_AF 3

#define AFDUCT_SENSOR_OPEN 0
#define AFDUCT_SENSOR_CLOSED 1

#define CARD_DUCT_SENSOR_OPEN 1
#define CARD_DUCT_SENSOR_CLOSED 0
#define DUCT_SENSOR_RESET 2

#define DUCT_OPEN 1
#define DUCT_CLOSED 2
#define DUCT_RESET 3

#define DUCT_LEVEL_LOW 1
#define DUCT_LEVEL_CORRECT 2
#define DUCT_LEVEL_HIGH 3

typedef struct SensorTypeDef{
	int8_t currentReading;
	int8_t presentState;
	uint8_t ductTimerIncrementBool;
	uint8_t ductSensorTimer;
	uint8_t ductStateChanged;
	uint8_t hysteresisTime;
	uint8_t deadTimeOn;
	uint8_t deadTime;
} Sensor;

extern Sensor ductCardFeed;
extern Sensor ductAutoFeed;
extern Sensor tgCoiler;


int8_t Sensor_ReadValueDirectly(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor);
uint8_t SensorAppyHysteresis(Sensor *s);

void setupSensorHysteresisTime(Sensor *s , uint8_t delayTime);
void setupSensorDeadTime(Sensor *s,uint8_t deadTime);

void SensorStartDeadTime(Sensor *s);
void SensorCheckDeadTimeOver(Sensor *s);
#endif /* MACHINESENSORS_H_ */

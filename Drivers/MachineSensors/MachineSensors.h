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
#define DUCTSENSOR_BTM_CARDFEED 2
#define DUCTSENSOR_AF 3

#define AFDUCT_SENSOR_OPEN 0
#define AFDUCT_SENSOR_CLOSED 1

#define DUCT_SENSOR_OPEN 1
#define DUCT_SENSOR_CLOSED 0
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
	uint16_t hysteresisTime;
} Sensor;

extern Sensor ductCardFeedTop;
extern Sensor ductCardFeedBtm;
extern Sensor ductAutoFeed;


int8_t Sensor_ReadValueDirectly(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor);
uint8_t SensorAppyHysteresis(Sensor *s);

void setupSensorHysteresisTime(Sensor *s , uint16_t delayTime);
void processCardFeedDuctLevel(CardingMc *c);

#endif /* MACHINESENSORS_H_ */

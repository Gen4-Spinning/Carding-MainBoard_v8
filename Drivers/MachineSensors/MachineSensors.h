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

#define DUCT_SENSOR_OPEN 1
#define DUCT_SENSOR_CLOSED 0

#define START_BTR_FEED 1
#define STOP_BTR_FEED 2
#define NOTHING_BUFFERED_BTR_FEED 0


#define SENSOR_ENABLE 1
#define SENSOR_DISABLE 2

typedef struct {
	int8_t ductSensor;
	int8_t ductCurrentState;
	uint8_t ductTimerIncrementBool;
	uint8_t ductSensorTimer;
	uint8_t ductStateChanged;

	uint8_t updateBtnPressed;
	uint8_t ductSensorDbgTimer;

	uint8_t coilerSensor_activated;
	uint8_t coilerSensor;
	uint8_t latchedCoilerSensor;

} SensorTypeDef;

extern SensorTypeDef sensor;

uint8_t Sensor_whichTriggered(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *whichSensor);
void Sensor_resetTriggeredStates(MCP23017_PortB *whichSensor);
int8_t Sensor_GetTriggerValue(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor);

void SetCoilerSensorState(SensorTypeDef *s,uint8_t state);

void DuctSensorMonitor(SensorTypeDef *s,machineSettingsTypeDef *msp);
uint8_t DuctSensor_CompareDuctStateWithBeaterFeedState(SensorTypeDef *s,RunTime_TypeDef *btrFeedData);

int8_t Sensor_ReadValueDirectly(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor);
uint8_t SensorAppyHysteresis(SensorTypeDef *s,int8_t sensorCurrentReading);

#endif /* MACHINESENSORS_H_ */

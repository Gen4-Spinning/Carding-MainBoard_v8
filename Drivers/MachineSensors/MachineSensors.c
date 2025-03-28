/*
 * MachineSensors.c
 *
 *  Created on: May 11, 2023
 *      Author: harsha
 */


#include "MachineSensors.h"
#include "machineSettings.h"


void setupSensorHysteresisTime(Sensor *s , uint8_t delayTime){
	s->hysteresisTime = delayTime;
}

void setupSensorDeadTime(Sensor *s,uint8_t deadTime){
	s->deadTime = deadTime;
}

int8_t Sensor_ReadValueDirectly(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor){
	mcp23017_read(mcp, MCP_GPIOB,mcp->intTriggerCapturedValue); // captures GPIO value when interrupt comes.
	sensorVal->raw = mcp->intTriggerCapturedValue[0];
	if (sensor == DUCTSENSOR_TOP_CARDFEED){
		return sensorVal->values.input0;
	}else if (sensor == TG_OPTICAL_SENSOR){
		return sensorVal->values.input1;
	}else if (sensor == DUCTSENSOR_AF){
		return sensorVal->values.input2;
	}
	return -1;
}


void SensorStartDeadTime(Sensor *s){
	s->ductSensorTimer = 0;
	s->ductTimerIncrementBool = 1;
	s->deadTimeOn = 1;
}

void SensorCheckDeadTimeOver(Sensor *s){
	if (s->deadTimeOn==1){
		if (s->ductSensorTimer >= s->deadTime ){
			s->deadTimeOn = 0;
		}
	}
}

uint8_t SensorAppyHysteresis(Sensor *s){
	// if sensor reading is different from the previous state.
	if (s->currentReading != s->presentState){
		if (s->ductTimerIncrementBool == 0){
			s->ductSensorTimer = 0;
			s->ductTimerIncrementBool = 1;
		}else{
			if (s->ductSensorTimer >= s->hysteresisTime ){
				s->presentState = s->currentReading;
				s->ductTimerIncrementBool = 0;
				s->ductSensorTimer = 0;
				return 1;		// apply the motor state and then reset these vars
			}else{
				return 0;
			}
		}
	}
	// if sensor reading is the same as the previous state,
	else{
		s->ductTimerIncrementBool = 0;
		s->ductSensorTimer = 0;
		return 0;
	}
	return 0;
}

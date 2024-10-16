/*
 * MachineSensors.c
 *
 *  Created on: May 11, 2023
 *      Author: harsha
 */


#include "MachineSensors.h"
#include "machineSettings.h"


void setupSensorHysteresisTime(Sensor *s , uint16_t delayTime){
	s->hysteresisTime = delayTime;
}

int8_t Sensor_ReadValueDirectly(MCP23017_HandleTypeDef *mcp, MCP23017_PortB *sensorVal,uint8_t sensor){
	mcp23017_read(mcp, MCP_GPIOB,mcp->intTriggerCapturedValue); // captures GPIO value when interrupt comes.
	sensorVal->raw = mcp->intTriggerCapturedValue[0];
	if (sensor == DUCTSENSOR_TOP_CARDFEED){
		return sensorVal->values.input0;
	}else if (sensor == DUCTSENSOR_BTM_CARDFEED){
		return sensorVal->values.input1;
	}else if (sensor == DUCTSENSOR_AF){
		return sensorVal->values.input2;
	}
	return -1;
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


void processCardFeedDuctLevel(CardingMc *c){
	if ((C.D.cardFeedTop_sensorState == DUCT_SENSOR_OPEN) && (C.D.cardFeedBtm_sensorState == DUCT_SENSOR_OPEN)){
		c->D.cardFeed_ductLevel = DUCT_LEVEL_LOW;
	}else if ((C.D.cardFeedTop_sensorState == DUCT_SENSOR_OPEN) && (C.D.cardFeedBtm_sensorState == DUCT_SENSOR_CLOSED)){
		c->D.cardFeed_ductLevel = DUCT_LEVEL_CORRECT;
	}else if ((C.D.cardFeedTop_sensorState == DUCT_SENSOR_CLOSED) && (C.D.cardFeedBtm_sensorState == DUCT_SENSOR_CLOSED)){
		c->D.cardFeed_ductLevel = DUCT_LEVEL_HIGH;
	}else{
		//top closed but Btm open. MomentaryBlip while cotton is crossing or mistake. So we run fast
		c->D.cardFeed_ductLevel = DUCT_LEVEL_LOW;
	}
}

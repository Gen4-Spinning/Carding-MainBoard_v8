/*
 * machineSettings.c
 *
 *  Created on: Apr 11, 2023
 *      Author: harsha
 */

#include "machineSettings.h"
#include "stdlib.h"

void InitInternalSettings(internalSettings *i){
	i->AF_ductSensorDelay = 2;
	i->cardingDuctSensor1Delay= 1;
	i->cardingDuctSensor2Delay= 1;
	i->piecingDeliveryMtrsMin = 3;
}

void setupCardingMCType(CardingMc *c,userSettings *u){

	c->delivery_mtrMin = u->delivery_mMin;
	c->lengthLimit = u->lengthLimit;
	c->tensionDraft = 1.02f; // hardcoded at beginning

	c->M.cardCylMotorRPM = u->cardCylRPM/CYLINDER_GEAR_RATIO;
	c->M.btrCylMotorRPM = u->btrCylRPM/CYLINDER_GEAR_RATIO;
	c->M.pickerCylMotorRPM = u->pickerCylRPM/AF_PICKER_CYL_GEAR_RATIO;

	c->R.cardFeedRPM = u->delivery_mMin/u->deliveryMtrMin_CardFeed_Ratio;
	if (c->R.cardFeedRPM > 11){c->R.cardFeedRPM = 11;}
	if (c->R.cardFeedRPM < 0.2){c->R.cardFeedRPM = 0.2;}
	c->M.cardFeedMotorRPM = c->R.cardFeedRPM * CYLINDER_FEED_GB;

	c->R.btrFeedRPM = c->R.cardFeedRPM * u->btrFeed_CardFeed_Ratio;
	if (c->R.btrFeedRPM > 11){c->R.btrFeedRPM = 11;}
	if (c->R.btrFeedRPM < 0.2){c->R.btrFeedRPM = 0.2;}
	c->M.btrFeedMotorRPM = c->R.btrFeedRPM * BEATER_FEED_GB;

	c->R.pickerFeedRPM = c->R.btrFeedRPM * u->afFeed_BtrFeed_Ratio;
	if (c->R.pickerFeedRPM > 8){c->R.pickerFeedRPM = 8;}
	if (c->R.pickerFeedRPM < 0.2){c->R.pickerFeedRPM = 0.2;}
	c->M.afFeedMotorRPM = c->R.pickerFeedRPM * AF_FEED_GB;

	c->R.TgRPM = (u->delivery_mMin*1000)/TONGUE_GROOVE_CIRCUMFERENCE_MM;
	float cageGB_shaftRPM = c->R.TgRPM  * TG_TO_GB_RATIO;
	c->R.cageRPM = cageGB_shaftRPM/CAGE_TO_GB_RATIO;
	c->M.cageMotorRPM =  cageGB_shaftRPM * CAGE_GB;

	float req_coiler_tongue_surfaceSpeed_mm = (u->delivery_mMin*1000) * c->tensionDraft ;
	float req_coiler_tongueRPM = req_coiler_tongue_surfaceSpeed_mm/COILER_GROOVE_CIRCUMFERENCE_MM;
	c->R.coilerGBShaftRPM = req_coiler_tongueRPM/COILER_GROOVE_TO_GB_RATIO;
	c->M.coilerMotorRPM = c->R.coilerGBShaftRPM  * COILER_GB;

}

void updateMachineSpeeds(CardingMc *c,userSettings *u,float deliveryMtrMin,uint8_t updateOnlyCardFeed){

	c->delivery_mtrMin = deliveryMtrMin;

	c->R.cardFeedRPM = c->delivery_mtrMin/u->deliveryMtrMin_CardFeed_Ratio;
	if (c->R.cardFeedRPM > 11){c->R.cardFeedRPM = 11;}
	if (c->R.cardFeedRPM < 0.2){c->R.cardFeedRPM = 0.2;}
	c->M.cardFeedMotorRPM = c->R.cardFeedRPM * CYLINDER_FEED_GB;

	if (updateOnlyCardFeed != 1){
		c->R.btrFeedRPM = c->R.cardFeedRPM * u->btrFeed_CardFeed_Ratio;
		if (c->R.btrFeedRPM > 11){c->R.btrFeedRPM = 11;}
		if (c->R.btrFeedRPM < 0.2){c->R.btrFeedRPM = 0.2;}
		c->M.btrFeedMotorRPM = c->R.btrFeedRPM * BEATER_FEED_GB;

		c->R.pickerFeedRPM = c->R.btrFeedRPM * u->afFeed_BtrFeed_Ratio;
		if (c->R.pickerFeedRPM > 8){c->R.pickerFeedRPM = 8;}
		if (c->R.pickerFeedRPM < 0.2){c->R.pickerFeedRPM = 0.2;}
		c->M.afFeedMotorRPM = c->R.pickerFeedRPM * AF_FEED_GB;
	}

	c->R.TgRPM = (c->delivery_mtrMin*1000)/TONGUE_GROOVE_CIRCUMFERENCE_MM;
	float cageGB_shaftRPM = c->R.TgRPM  * TG_TO_GB_RATIO;
	c->R.cageRPM = cageGB_shaftRPM/CAGE_TO_GB_RATIO;
	c->M.cageMotorRPM =  cageGB_shaftRPM * CAGE_GB;

	float req_coiler_tongue_surfaceSpeed_mm = (c->delivery_mtrMin*1000) * c->tensionDraft ;
	float req_coiler_tongueRPM = req_coiler_tongue_surfaceSpeed_mm/COILER_GROOVE_CIRCUMFERENCE_MM;
	c->R.coilerGBShaftRPM = req_coiler_tongueRPM/COILER_GROOVE_TO_GB_RATIO;
	c->M.coilerMotorRPM = c->R.coilerGBShaftRPM  * COILER_GB;
}



void CalculateMachineParameters(machineSettingsTypeDef *ms,machineParamsTypeDef *m){
	m->cylinderRPM = ms->cylinderSpeed/CYLINDER_GEAR_RATIO;
	m->beaterRPM = ms->beaterSpeed/CYLINDER_GEAR_RATIO;
	m->cylinderFeedRPM = ms->cylinderFeed * CYLINDER_FEED_GB;
	m->beaterFeedRPM = ms->beaterFeed * BEATER_FEED_GB;
	m->cageGB_RPM = (ms->delivery_mMin*1000)/TONGUE_GROOVE_CIRCUMFERENCE_MM;
	m->cageRPM = m->cageGB_RPM * CAGE_GB;
	float req_coiler_tongue_surfaceSpeed_mm = (ms->delivery_mMin*1000) * ms->draft;
	float req_coiler_tongueRPM = req_coiler_tongue_surfaceSpeed_mm/COILER_GROOVE_CIRCUMFERENCE_MM;
	m->coilerGB_rpm = req_coiler_tongueRPM/COILER_GROOVE_TO_GB_RATIO;
	m->coilerRPM = m->coilerGB_rpm * COILER_GB;
}

void UpdateMachineParameters(machineSettingsTypeDef *ms,machineParamsTypeDef *m){
	m->cylinderFeedRPM = ms->cylinderFeed * CYLINDER_FEED_GB;
	m->beaterFeedRPM = ms->beaterFeed * BEATER_FEED_GB;
	m->cageGB_RPM = (ms->delivery_mMin*1000)/TONGUE_GROOVE_CIRCUMFERENCE_MM;
	m->cageRPM = m->cageGB_RPM * CAGE_GB;
	float req_coiler_tongue_surfaceSpeed_mm = (ms->delivery_mMin*1000) * ms->draft;
	float req_coiler_tongueRPM = req_coiler_tongue_surfaceSpeed_mm/COILER_GROOVE_CIRCUMFERENCE_MM;
	m->coilerGB_rpm = req_coiler_tongueRPM/COILER_GROOVE_TO_GB_RATIO;
	m->coilerRPM = m->coilerGB_rpm * COILER_GB;
}

//for coiler Tension draft pot calcs.the coiler RPM at deliverym/min and draft of 1
uint16_t calcBaseCoilerRPM(userSettings *u){
	float req_coiler_tongue_surfaceSpeed_mm = (u->delivery_mMin*1000) * 1;
	float req_coiler_tongueRPM = req_coiler_tongue_surfaceSpeed_mm/COILER_GROOVE_CIRCUMFERENCE_MM;
	float coilerGB_rpm = req_coiler_tongueRPM/COILER_GROOVE_TO_GB_RATIO;
	uint16_t coilerRPM = coilerGB_rpm * COILER_GB;
	return coilerRPM;
}

void updateCoilerParameters(machineSettingsTypeDef *ms,machineParamsTypeDef *m){
		float req_coiler_tongue_surfaceSpeed_mm = (ms->delivery_mMin*1000) * ms->draft;
		float req_coiler_tongueRPM = req_coiler_tongue_surfaceSpeed_mm/COILER_GROOVE_CIRCUMFERENCE_MM;
		m->coilerGB_rpm = req_coiler_tongueRPM/COILER_GROOVE_TO_GB_RATIO;
		m->coilerRPM = m->coilerGB_rpm * COILER_GB;
}

uint8_t getMotorCANAddress(uint8_t motor){
	if (motor == CARDING_CYLINDER){
		return CARDING_CYLINDER_CAN_ADDRESS;
	}else if (motor == BEATER_CYLINDER){
		return BEATER_CYLINDER_CAN_ADDRESS;
	}else if (motor == CAGE){
		return CAGE_CAN_ADDRESS;
	}else if (motor == CARDING_FEED){
		return CARDFEED_CAN_ADDRESS;
	}else if (motor == BEATER_FEED){
		return BTRFEED_CAN_ADDRESS;
	}else if (motor == COILER){
		return COILER_CAN_ADDRESS;
	}else if (motor == AF_PICKER_CYLINDER){
		return PICKERCYL_CAN_ADDRESS;
	}else if (motor == AF_FEED){
		return AF_FEED_CAN_ADDRESS;
	}else{
		return 0;
	}
}

uint8_t GetMotorID_from_CANAddress(uint8_t canAddress){
	if (canAddress == CARDING_CYLINDER_CAN_ADDRESS){
		return CARDING_CYLINDER;
	}else if (canAddress == BEATER_CYLINDER_CAN_ADDRESS){
		return BEATER_CYLINDER;
	}else if (canAddress == CAGE_CAN_ADDRESS){
		return CAGE;
	}else if (canAddress == CARDFEED_CAN_ADDRESS){
		return CARDING_FEED;
	}else if (canAddress == BTRFEED_CAN_ADDRESS){
		return BEATER_FEED;
	}else if (canAddress == COILER_CAN_ADDRESS){
		return COILER;
	}else if (canAddress == PICKERCYL_CAN_ADDRESS){
		return AF_PICKER_CYLINDER;
	}else if (canAddress == AF_FEED_CAN_ADDRESS){
		return AF_FEED;
	}
	return 8; // return
}


uint8_t CheckCylindersRampUpOver(CardingMc *c,ExtendedRunTime_TypeDef *cylinder,ExtendedRunTime_TypeDef *beater,RunTime_TypeDef *pickerCylinder){
	if (abs(cylinder->actualRPM) >= (c->M.cardCylMotorRPM - 20)){
		if (abs(beater->actualRPM) >= (c->M.btrCylMotorRPM - 20)){
			if (abs(pickerCylinder->presentRPM) >= (c->M.pickerCylMotorRPM)){
			return 1;
			}
		}
	}
	return 0;
}


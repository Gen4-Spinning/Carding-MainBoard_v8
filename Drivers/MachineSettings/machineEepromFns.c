/*
 * EepromFns.c
 *
 *  Created on: 06-Mar-2023
 *      Author: harsha
 */

#include "Eeprom.h"
#include "machineSettings.h"
#include "machineEepromSettings.h"

void ReadUserSettingsFromEeprom(userSettings *u){
	u->delivery_mMin = EE_ReadFloat(C_DELIVERY_M_MIN_ADDR);
	u->lengthLimit = EE_ReadInteger(C_LENGTH_LIMIT_M_ADDR);
	//--advanced Settings---
	u->cardCylRPM = EE_ReadInteger(C_CARDING_CYL_SPEED_ADDR);
	u->btrCylRPM = EE_ReadInteger(C_BEATER_CYL_SPEED_ADDR);
	u->pickerCylRPM = EE_ReadInteger(C_PICKER_CYL_SPEED_ADDR);
	//transferRatios
	u->deliveryMtrMin_CardFeed_Ratio =  EE_ReadFloat(C_DELIVERY_CARDFEED_RATIO_ADDR);
	u->btrFeed_CardFeed_Ratio =  EE_ReadFloat(C_BTRFEED_CARDFEED_RATIO_ADDR);
	u->afFeed_BtrFeed_Ratio =  EE_ReadFloat(C_AFFEED_BTRFEED_RATIO_ADDR);
}

uint8_t CheckUserSettings(userSettings* u){
	//typically when something goes wrong with the eeprom you get a value that is very high..
	if ((u->delivery_mMin > 50 ) || (u->delivery_mMin < 2.5)){
		return 0;
	}
	if ((u->lengthLimit > 1000)||(u->lengthLimit < 100)){
		return 0;
	}
	if ((u->cardCylRPM > 750)||(u->cardCylRPM < 300)){
		return 0;
	}
	if ((u->btrCylRPM > 650) || (u->btrCylRPM < 300)){
		return 0;
	}
	if ((u->pickerCylRPM > 650) || (u->pickerCylRPM < 300)){
		return 0;
	}
	if ((u->deliveryMtrMin_CardFeed_Ratio > 3.0f) || (u->deliveryMtrMin_CardFeed_Ratio < 10.0f)){
			return 0;
	}
	if ((u->btrFeed_CardFeed_Ratio > 0.2f) || (u->btrFeed_CardFeed_Ratio < 2.0)){
			return 0;
	}
	if ((u->afFeed_BtrFeed_Ratio > 0.2f) || (u->afFeed_BtrFeed_Ratio < 2.0f)){
			return 0;
	}
	return 1;

}


void LoadDefaultUserSettings(userSettings *u){
	u->delivery_mMin = DEFAULT_DELIVERY_M_MIN;
	u->lengthLimit = DEFAULT_LENGTH_LIMIT;
	u->cardCylRPM = DEFAULT_CARDING_CYL_SPEED;
	u->btrCylRPM = DEFAULT_BEATER_CYL_SPEED;
	u->pickerCylRPM = DEFAULT_PICKER_CYL_SPEED;
	u->deliveryMtrMin_CardFeed_Ratio = DEFAULT_DELIVERY_CARDFEED_RATIO;
	u->btrFeed_CardFeed_Ratio = DEFAULT_BTRFEED_CARDFEED_RATIO;
	u->afFeed_BtrFeed_Ratio = DEFAULT_AFFEED_BTRFEED_RATIO;
}


uint8_t WriteUserSettingsIntoEeprom(userSettings *u){
	uint8_t dataWritten = 0;
    dataWritten += EE_WriteFloat(u->delivery_mMin,C_DELIVERY_M_MIN_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(u->lengthLimit,C_LENGTH_LIMIT_M_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(u->cardCylRPM,C_CARDING_CYL_SPEED_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(u->btrCylRPM,C_BEATER_CYL_SPEED_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(u->pickerCylRPM,C_PICKER_CYL_SPEED_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteFloat(u->deliveryMtrMin_CardFeed_Ratio,C_DELIVERY_CARDFEED_RATIO_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteFloat(u->btrFeed_CardFeed_Ratio,C_BTRFEED_CARDFEED_RATIO_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteFloat(u->afFeed_BtrFeed_Ratio,C_AFFEED_BTRFEED_RATIO_ADDR);
    HAL_Delay(2);
    if (dataWritten == 8)
    	{return 0;}
    else{
    	return 1;} //return 1 is failure
}



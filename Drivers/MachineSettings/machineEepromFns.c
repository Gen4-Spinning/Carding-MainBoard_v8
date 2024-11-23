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
	u->btrFeedRPM =  EE_ReadFloat(C_BTRFEED_SPEED_ADDR);
	u->AF_FeedRPM =  EE_ReadFloat(C_AFFEED_SPEED_ADDR);
	//transferRatios
	u->deliveryMtrMin_CardFeed_Ratio =  EE_ReadFloat(C_DELIVERY_CARDFEED_RATIO_ADDR);

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
	if ((u->btrFeedRPM > 11.0f) || (u->btrFeedRPM < 0.1f)){
			return 0;
	}
	if ((u->AF_FeedRPM > 8.0f) || (u->AF_FeedRPM < 0.1f)){
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
	u->btrFeedRPM = DEFAULT_BTRFEED_SPEED;
	u->AF_FeedRPM = DEFAULT_AFFEED_SPEED;
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
    dataWritten += EE_WriteInteger(u->btrFeedRPM,C_BTRFEED_SPEED_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteInteger(u->AF_FeedRPM,C_AFFEED_SPEED_ADDR);
    HAL_Delay(2);
    dataWritten += EE_WriteFloat(u->deliveryMtrMin_CardFeed_Ratio,C_DELIVERY_CARDFEED_RATIO_ADDR);
    HAL_Delay(2);
    if (dataWritten == 8)
    	{return 0;}
    else{
    	return 1;} //return 1 is failure
}



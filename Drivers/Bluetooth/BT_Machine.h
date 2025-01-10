/*
 * BT_Constants.h
 *
 *  Created on: Mar 9, 2023
 *      Author: harsha
 */

#ifndef BT_MC_H_
#define BT_MC_H_

#include "BT_Fns.h"
#include "machineSettings.h"

//settings TLVs
#define DELIVERY_M_MIN_BT 0x80
#define CARD_FEED_RATIO_BT 0x81
#define LENGTH_LIMIT_BT 0x82
#define CARDING_CYL_SPEED_BT 0x83
#define BEATER_CYL_SPEED_BT 0x84
#define PICKER_CYL_SPEED_BT 0x85
#define BEATER_FEED_SPEED_BT 0x86
#define AF_FEED_SPEED_BT 0x87

#define RAMPTIMES_BT 0x88

#define BT_CARDING_CYLINDER 0x01
#define BT_BEATER_CYLINDER 0x02
#define BT_CAGE 0x03
#define BT_CARDING_FEED 0x04
#define BT_BEATER_FEED 0x05
#define BT_COILER 0x06
#define BT_PICKER_CYL 0x07
#define BT_AF_FEED 0x08

#define BT_DIAGNOSTIC_RUN 0x01
#define BT_DIAGNOSTIC_STOP 0x06

#define SAVE_CARDING_SETTINGS 0
#define UPDATE_CARDING_SETTINGS 1

uint8_t BT_MC_generateSettingsMsg(userSettings *u);
uint8_t BT_MC_parse_Settings(userSettings *uspBT);
uint8_t BT_MC_Save_Settings(userSettings *u);
uint8_t BT_MC_Update_Settings(userSettings *u,userSettings *uBT);
uint8_t GetMotorID_from_BTMotor_ID(uint8_t BT_motorID);
uint8_t GetMotorId_from_CarousalID(uint8_t carousalID);
uint8_t GetBTMotorID_from_Motor_ID(uint8_t motorID);

#endif /* BT_MC_H_ */

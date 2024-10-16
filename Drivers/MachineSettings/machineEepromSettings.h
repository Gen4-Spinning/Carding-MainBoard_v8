/*
 * EepromSettings.h
 *
 *  Created on: 06-Mar-2023
 *      Author: harsha
 */

#ifndef INC_EEPROMSETTINGS_H_
#define INC_EEPROMSETTINGS_H_

//Addresses
//Dont let a address go across address 32 and its multiples. Thats one page.
#define C_DELIVERY_M_MIN_ADDR 0X02 //float
#define C_LENGTH_LIMIT_M_ADDR 0X06 // int
#define C_CARDING_CYL_SPEED_ADDR 0x08 // int
#define C_BEATER_CYL_SPEED_ADDR 0x0A // int
#define C_PICKER_CYL_SPEED_ADDR 0x0C // int
#define C_BTRFEED_SPEED_ADDR 0x10 // int
#define C_AFFEED_SPEED_ADDR 0x12 // int
#define C_DELIVERY_CARDFEED_RATIO_ADDR 0x14// float


//DEFAULTS
#define DEFAULT_DELIVERY_M_MIN 8
#define DEFAULT_LENGTH_LIMIT 100
#define DEFAULT_CARDING_CYL_SPEED 750
#define DEFAULT_BEATER_CYL_SPEED 600
#define DEFAULT_PICKER_CYL_SPEED 500
#define DEFAULT_BTRFEED_SPEED 5.0
#define DEFAULT_AFFEED_SPEED 5.0
#define DEFAULT_DELIVERY_CARDFEED_RATIO 5


#define DEFAULT_RAMP_TIMES 5	// in sec


#endif /* INC_EEPROMSETTINGS_H_ */

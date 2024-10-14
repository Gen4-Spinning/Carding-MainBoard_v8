/*
 * TD_Pot.h
 *
 *  Created on: Jul 20, 2024
 *      Author: harsha
 */

#ifndef TENSIONDRAFT_POT_TD_POT_H_
#define TENSIONDRAFT_POT_TD_POT_H_

#define MIN_TD_DRAFT  1.0f
#define MAX_TD_DRAFT  2.0f
#define TD_BANDS_NO  10.0f
#define TD_BAND_DELTA_ADC 409 // set this based on TD_BANDS_NO (4095/bandNo)
#define TD_CHANGEDELAY_THRESH 1

#define TD_MAX_COILER_RPM 1350

#include "main.h"
#include "machineSettings.h"

extern ADC_HandleTypeDef hadc2;

typedef struct  TD_ADC{
	uint16_t rawADCval;

	float usedDeliveryM_min;
	uint16_t baseCoilerRPM;
	float td_maxDraft;
	float td_band_delta_draft; //delta draft per each band



	uint8_t instReadingLevel;
	uint8_t appliedLevel;

	uint8_t td_changeCounter;
	uint8_t td_changedBool;

	float tensionDraft;
	uint8_t tensionDraftChanged;


	uint8_t beepEnable;
	uint8_t beepCounter;
}TDP;


void TD_readADC(TDP *tdp);
void TD_calculate(TDP *tdp);
void TD_calculateMaxDraft(TDP *tdp,userSettings *u);

extern TDP tdp;



#endif /* TENSIONDRAFT_POT_TD_POT_H_ */

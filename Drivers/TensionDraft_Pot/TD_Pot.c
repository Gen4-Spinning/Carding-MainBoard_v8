/*
 * TD_Pot.c
 *
 *  Created on: Jul 20, 2024
 *      Author: harsha
 */

#include  "TD_Pot.h"


void TD_readADC(TDP *tdp){
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	tdp->rawADCval = HAL_ADC_GetValue(&hadc2);
	if (tdp->rawADCval > 4000){ // so that we dont jump around 4095
		tdp->rawADCval = 4095;
	}
}

/* here we calculate the max tension draft possible given the delivery mtrs per min
 * of the machine
 */
void TD_calculateMaxDraft(TDP *tdp,userSettings *u){
		tdp->baseCoilerRPM = calcBaseCoilerRPM(*(&u));
		tdp->td_maxDraft = (float)TD_MAX_COILER_RPM/tdp->baseCoilerRPM;
		if (tdp->td_maxDraft > MAX_TD_DRAFT){
			tdp->td_maxDraft = MAX_TD_DRAFT;
		}
		tdp->td_band_delta_draft = (tdp->td_maxDraft - 1.0f)/TD_BANDS_NO;
		tdp->usedDeliveryM_min = u->delivery_mMin;
}

/* Here we check if the adc value has changed, and if it has we wait for a few iterations
 * of this loop to see if it has again changed. if it has we reset the iteration counter
 * and wait again. finally when the iteration counter reaches a threshold, we apply
 * the new value.
 */
void TD_calculate(TDP *tdp){
	tdp->instReadingLevel = tdp->rawADCval/TD_BAND_DELTA_ADC;
	if (tdp->appliedLevel != tdp->instReadingLevel){ // keep it ints applies some hysteresis automatically.
		tdp->appliedLevel = tdp->instReadingLevel;
		tdp->td_changeCounter=0;
		tdp->td_changedBool=1;
	}else{
		if (tdp->td_changedBool){
			tdp->td_changeCounter++;
			if (tdp->td_changeCounter == TD_CHANGEDELAY_THRESH){
			tdp->tensionDraft = MIN_TD_DRAFT + (float)tdp->appliedLevel * tdp->td_band_delta_draft;
				tdp->tensionDraftChanged = 1;
				tdp->td_changedBool = 0;
				tdp->td_changeCounter = 0;
			}
		}else{
			tdp->td_changedBool = 0;
		}
	}
}






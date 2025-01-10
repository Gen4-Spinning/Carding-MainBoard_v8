/*
 * BT_Machine.c
 *
 *  Created on: 25-Apr-2023
 *      Author: harsha
 */

#include "BT_Machine.h"


uint8_t BT_MC_generateSettingsMsg(userSettings *u){
	  char TLV_Buffer[12];
	  uint8_t tlvSize = 0;
	  uint8_t eof_size  = 0;
	  uint8_t initLength = 0;

	  initLength = Init_TXBuf_Frame(SETTINGS_FROM_MC,SUBSTATE_NA,12);

	  generateTLV_F(TLV_Buffer,DELIVERY_M_MIN_BT,u->delivery_mMin);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_FLOAT,initLength+tlvSize);
	  tlvSize += TLV_FLOAT;

	  generateTLV_F(TLV_Buffer,CARD_FEED_RATIO_BT,u->deliveryMtrMin_CardFeed_Ratio);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_FLOAT,initLength+tlvSize);
	  tlvSize += TLV_FLOAT;

	  generateTLV_I(TLV_Buffer,LENGTH_LIMIT_BT,(uint16_t)u->lengthLimit);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_I(TLV_Buffer,CARDING_CYL_SPEED_BT,u->cardCylRPM);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_I(TLV_Buffer,BEATER_CYL_SPEED_BT,u->btrCylRPM);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_I(TLV_Buffer,PICKER_CYL_SPEED_BT,u->pickerCylRPM);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_I(TLV_Buffer,BEATER_FEED_SPEED_BT,u->btrFeedRPM);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  generateTLV_I(TLV_Buffer,AF_FEED_SPEED_BT,u->AF_FeedRPM);
	  add_TLVBuf_To_TxBuf(TLV_Buffer,TLV_INT,initLength+tlvSize);
	  tlvSize += TLV_INT;

	  eof_size = addEOF(initLength+tlvSize);
	  correctLengthInFrame(initLength,tlvSize,eof_size);

	  return initLength + tlvSize + eof_size;

}

//FLYER
uint8_t BT_MC_parse_Settings(userSettings *uspBT){
	//Buffer Rec index 10 onwards is TLVs till 10 + TlvsLength
	TLVStruct_TypeDef T;
	uint8_t TLV_start = 10;
	uint8_t tlvSize = 0;
	uint8_t count = 0;
	uint8_t allSettingsRecieved = 0;

    for (int i=0;i<BT.attributeCnt;i++){
    	tlvSize = parseTLV(&T,TLV_start);
    	TLV_start += tlvSize;
    	switch (T.type){
    		case DELIVERY_M_MIN_BT:
    			uspBT->delivery_mMin = T.value_f;
    			count += 1;
    			break;
    		case CARD_FEED_RATIO_BT:
    			uspBT->deliveryMtrMin_CardFeed_Ratio = T.value_f;
    			count += 1;
    			break;
    		case LENGTH_LIMIT_BT:
    			uspBT->lengthLimit = T.value_int;
    			count += 1;
    			break;
    		case CARDING_CYL_SPEED_BT:
    			uspBT->cardCylRPM = T.value_int;
    			count += 1;
    			break;
    		case BEATER_CYL_SPEED_BT:
    			uspBT->btrCylRPM = T.value_int;
    			count += 1;
    			break;
    		case PICKER_CYL_SPEED_BT:
    			uspBT->pickerCylRPM = T.value_int;
    			count += 1;
    			break;
    		case BEATER_FEED_SPEED_BT:
    			uspBT->btrFeedRPM = T.value_int;
    			count += 1;
    			break;
       		case AF_FEED_SPEED_BT:
       			uspBT->AF_FeedRPM = T.value_int;
				count += 1;
				break;
    	}
    }
    if (count == 8){
    	allSettingsRecieved = 1;
    }

    return allSettingsRecieved;
}

//FLYER
uint8_t BT_MC_Save_Settings(userSettings *u){
	uint8_t fail;
	fail = WriteUserSettingsIntoEeprom(u);
	return !fail;
}

uint8_t BT_MC_Update_Settings(userSettings *u,userSettings *uBT){
	u->delivery_mMin = uBT->delivery_mMin;
	u->deliveryMtrMin_CardFeed_Ratio = uBT->deliveryMtrMin_CardFeed_Ratio;
	u->lengthLimit = uBT->lengthLimit;
	u->cardCylRPM = uBT->cardCylRPM;
	u->btrCylRPM = uBT->btrCylRPM;
	u->pickerCylRPM = uBT->pickerCylRPM;
	u->btrFeedRPM = uBT->btrFeedRPM;
	u->AF_FeedRPM = uBT->AF_FeedRPM;
	//send success msg to APP
	return 1;
}

uint8_t GetMotorID_from_BTMotor_ID(uint8_t BT_motorID){
	if (BT_motorID == BT_CARDING_CYLINDER){
		return CARDING_CYLINDER;
	}else if (BT_motorID == BT_BEATER_CYLINDER){
		return BEATER_CYLINDER;
	}else if (BT_motorID == BT_CAGE){
		return CAGE;
	}else if (BT_motorID == BT_CARDING_FEED){
		return CARDING_FEED;
	}else if (BT_motorID == BT_BEATER_FEED){
		return BEATER_FEED;
	}else if (BT_motorID == BT_COILER){
		return COILER;
	}else if (BT_motorID == BT_PICKER_CYL){
		return AF_PICKER_CYLINDER;
	}else if (BT_motorID == BT_AF_FEED){
		return AF_FEED;
	}
	return 0;
}

uint8_t GetBTMotorID_from_Motor_ID(uint8_t motorID){
	if (motorID == CARDING_CYLINDER){
		return BT_CARDING_CYLINDER;
	}else if (motorID == BEATER_CYLINDER){
		return BT_BEATER_CYLINDER;
	}else if (motorID == CAGE){
		return BT_CAGE;
	}else if (motorID == CARDING_FEED){
		return BT_CARDING_FEED;
	}else if (motorID == BEATER_FEED){
		return BT_BEATER_FEED;
	}else if (motorID == COILER){
		return BT_COILER;
	}else if (motorID == AF_PICKER_CYLINDER){
		return BT_PICKER_CYL;
	}else if (motorID == AF_FEED){
		return BT_AF_FEED;
	}
	return 0;
}

uint8_t GetMotorId_from_CarousalID(uint8_t carousalID){
	if (carousalID == BT_CARDING_CYLINDER){
		return CARDING_CYLINDER;
	}else if (carousalID == BT_BEATER_CYLINDER){
		return BEATER_CYLINDER;
	}else if (carousalID == BT_CAGE){
		return CAGE;
	}else if (carousalID == BT_CARDING_FEED){
		return CARDING_FEED;
	}else if (carousalID == BT_BEATER_FEED){
		return BEATER_FEED;
	}else if (carousalID == BT_COILER){
		return COILER;
	}else if (carousalID == BT_PICKER_CYL){
		return AF_PICKER_CYLINDER;
	}else if (carousalID == BT_AF_FEED){
		return AF_FEED;
	}
	return 99;
}


/*
 * AC_SSR.c
 *
 *  Created on: 17-Jul-2024
 *      Author: harsha
 */


#include "AC_SSR.h"

void setSSR_State(uint8_t state,MCP23017_HandleTypeDef *mcp,MCP23017_PortA *p){
	p->raw = 0;
	p->values.AC_SSR = state;
	mcp->outputGPIO[0] = p->raw;
	mcp23017_write(mcp, OLATA,mcp->outputGPIO);
}

uint8_t CheckAC_SSR(uint8_t state,MCP23017_HandleTypeDef *mcp,MCP23017_PortA *p){
	mcp23017_read(mcp, MCP_GPIOA,mcp->outputGPIO);
	p->raw = mcp->outputGPIO[0];
	if (p->values.AC_SSR == state){
		return 1;
	}else{
		return 0;
	}
}


/*
 * AC_SSR.h
 *
 *  Created on: 17-Jul-2024
 *      Author: harsha
 */

#include "mcp23017.h"

#ifndef AC_SSR_H_
#define AC_SSR_H_

#define AC_SSR_ON 1
#define AC_SSR_OFF 0

void setSSR_State(uint8_t state,MCP23017_HandleTypeDef *mcp,MCP23017_PortA *p);
uint8_t CheckAC_SSR(uint8_t state,MCP23017_HandleTypeDef *mcp,MCP23017_PortA *p);


#endif /* AC_SSR_AC_SSR_H_ */

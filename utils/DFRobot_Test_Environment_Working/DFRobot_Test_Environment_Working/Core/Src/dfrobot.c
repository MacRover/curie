/*
 * dfrobot.c
 *
 *  Created on: Sep 29, 2026
 *      Author: zokur
 */

#include "dfrobot.h"

DFROBOT_RS485_HandleTypeDef dfrobot_rs485;


void DFROBOT_INIT(void) {

    if (DFROBOT_RS485_Init(&dfrobot_rs485, &huart3) != DFROBOT_RS485_OK) {
    	Error_Handler();
    }

}


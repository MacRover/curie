/*
 * DFRobot.h
 *
 *  Created on: Sep 29, 2026
 *      Author: Gobind and Adam
 *
 */

#ifndef SRC_DFROBOT_H_
#define SRC_DFROBOT_H_

#include "main.h"
#include <dfrobot_rs485.h>

// Again I hope we can get away without this'

extern DFROBOT_RS485_HandleTypeDef dfrobot_rs485;
extern UART_HandleTypeDef huart3;

void DFROBOT_INIT(void);



#endif /* SRC_DFROBOT_H_ */

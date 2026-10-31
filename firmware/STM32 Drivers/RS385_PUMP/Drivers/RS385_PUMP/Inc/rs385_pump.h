/*
 * rs385_pump.h
 *
 *  Created on: 30-Oct-2026
 *      Author: muhammadhasan
 */

#ifndef RS385_PUMP_INC_RS385_PUMP_H_
#define RS385_PUMP_INC_RS385_PUMP_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h> // TODO adam check this


typedef enum {
    RS385_PUMP_OK,
    RS385_PUMP_ERROR,
    RS385_PUMP_BUSY,
    RS385_PUMP_TIMEOUT  
} RS385_PUMP_StatusTypeDef;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin; // register is 16 bit wide
    bool is_active;
} RS385_PUMP_HandleTypeDef;


RS385_PUMP_StatusTypeDef RS385_PUMP_PumpEnable(RS385_PUMP_HandleTypeDef *pump);
RS385_PUMP_StatusTypeDef RS385_PUMP_PumpDisable(RS385_PUMP_HandleTypeDef *pump);
RS385_PUMP_StatusTypeDef RS385_PUMP_IsPumpActive(RS385_PUMP_HandleTypeDef *pump);
#endif /* RS385_PUMP_INC_RS385_PUMP_H_ */

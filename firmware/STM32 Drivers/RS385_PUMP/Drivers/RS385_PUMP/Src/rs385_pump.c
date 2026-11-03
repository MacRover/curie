/*
 * rs385_pump.c
 *
 *  Created on: 30-Oct-2026
 *      Author: muhammadhasan
 */

//#ifndef RS385_PUMP_SRC_RS385_PUMP_C_
//#define RS385_PUMP_SRC_RS385_PUMP_C_

#include "rs385_pump.h"



//#endif /* RS385_PUMP_SRC_RS385_PUMP_C_ */

RS385_PUMP_StatusTypeDef RS385_PUMP_PumpEnable(RS385_PUMP_HandleTypeDef *pump) {
    HAL_GPIO_WritePin(pump->port, pump->pin, GPIO_PIN_SET); // PULLS HIGH
    return RS385_PUMP_OK;

}

RS385_PUMP_StatusTypeDef RS385_PUMP_PumpDisable(RS385_PUMP_HandleTypeDef *pump) {
    HAL_GPIO_WritePin(pump->port, pump->pin, GPIO_PIN_RESET); // PULLS LOW
    return RS385_PUMP_OK;

}
RS385_PUMP_StatusTypeDef RS385_PUMP_IsPumpActive(RS385_PUMP_HandleTypeDef *pump) {
    if (HAL_GPIO_ReadPin(pump->port, pump->pin) == GPIO_PIN_SET) {
        pump->is_active = true;
    } else {
        pump->is_active = false;
    }

    return RS385_PUMP_OK;
}

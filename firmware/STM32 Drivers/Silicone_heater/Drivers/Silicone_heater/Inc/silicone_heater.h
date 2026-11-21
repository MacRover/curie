/*
 * silicone_heater.h
 *
 *  Created on: 20-Nov-2026
 *      Author: muhammadhasan
 */

#ifndef SILICONE_HEATER_INC_SILICONE_HEATER_H_
#define SILICONE_HEATER_INC_SILICONE_HEATER_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h> 


typedef enum {
    SILICONE_HEATER_OK,
    SILICONE_HEATER_ERROR,
    SILICONE_HEATER_BUSY,
    SILICONE_HEATER_TIMEOUT
} SILICONE_HEATER_StatusTypeDef;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    bool is_active;
} SILICONE_HEATER_HandleTypeDef;


SILICONE_HEATER_StatusTypeDef SILICONE_HEATER_Enable(SILICONE_HEATER_HandleTypeDef *heater);
SILICONE_HEATER_StatusTypeDef SILICONE_HEATER_Disable(SILICONE_HEATER_HandleTypeDef *heater);


#endif /* SILICONE_HEATER_INC_SILICONE_HEATER_H_ */

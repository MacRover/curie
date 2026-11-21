/*
 * Silicone_heater.c
 *
 *  Created on: 20-Nov-2026
 *      Author: muhammadhasan
 */


#include "silicone_heater.h"

SILICONE_HEATER_StatusTypeDef SILICONE_HEATER_Enable(
    SILICONE_HEATER_HandleTypeDef *heater)
{
    HAL_GPIO_WritePin(heater->port, heater->pin, GPIO_PIN_SET);
    heater->is_active = true;
    return SILICONE_HEATER_OK;
}

SILICONE_HEATER_StatusTypeDef SILICONE_HEATER_Disable(
    SILICONE_HEATER_HandleTypeDef *heater)
{
    HAL_GPIO_WritePin(heater->port, heater->pin, GPIO_PIN_RESET);
    heater->is_active = false;
    return SILICONE_HEATER_OK;
}
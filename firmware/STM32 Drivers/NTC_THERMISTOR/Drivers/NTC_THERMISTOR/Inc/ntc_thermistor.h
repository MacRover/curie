/*
 * ntc_thermistor.h
 *
 *  Created on: Oct 28, 2026
 *      Author: Adam
 */

#ifndef NTC_THERMISTOR_INC_NTC_THERMISTOR_H_
#define NTC_THERMISTOR_INC_NTC_THERMISTOR_H_

// TODO: Change back to f1xx for onboard implementation
#include "stm32f1xx_hal.h"
#include <math.h>

#define KELVIN_CELCIUS_CONVERSION 273.15f
#define ROOM_TEMPERATURE_KELVIN 298.15f
#define ADC_MAX 4095.0f

typedef enum {
	NTC_THERMISTOR_OK,
	NTC_THERMISTOR_ERROR,
	NTC_THERMISTOR_BUSY,
	NTC_THERMISTOR_TIMEOUT
} NTC_THERMISTOR_StatusTypeDef;

typedef enum {
	ADC_OK,
	ADC_ERROR,
	ADC_BUSY,
	ADC_TIMEOUT
} NTC_THERMISTOR_ADCStatusTypeDef;

typedef struct {
	float fixed_resistance;
	float resistance_25c;
	float resistance_thermistor;
	float temp;
	float beta;

	uint32_t adc_raw;
} NTC_THERMISTOR_HandleTypeDef;

// Functions needed

float fast_ln(float x);
NTC_THERMISTOR_ADCStatusTypeDef NTC_THERMISTOR_ReadADCValue(NTC_THERMISTOR_HandleTypeDef *device, ADC_HandleTypeDef *hadc);
NTC_THERMISTOR_StatusTypeDef NTC_THERMISTOR_GetResistance(NTC_THERMISTOR_HandleTypeDef *device);
NTC_THERMISTOR_StatusTypeDef NTC_THERMISTOR_GetTempCelcius(NTC_THERMISTOR_HandleTypeDef *device);
NTC_THERMISTOR_StatusTypeDef NTC_THERMISTOR_OrchestratorFunction(NTC_THERMISTOR_HandleTypeDef *device, ADC_HandleTypeDef *hadc);



#endif /* NTC_THERMISTOR_INC_NTC_THERMISTOR_H_ */

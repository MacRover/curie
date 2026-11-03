/*
 * ntc_thermistor.c
 *
 *  Created on: Oct 28, 2026
 *      Author: Adam
 */

#include "ntc_thermistor.h"

float fast_ln(float x) {
	// Lowkey this function was entirely given to me by AI
	// If you think I like Taylor series, think again
	// However this does intuitively make sense
	// It's basically doing a Taylor series expansion and we are taking it on faith as a fast ln

	// I mean I just put it into python versus ln and it works

	int exp;
	float m = frexpf(x, &exp);

	// Range reduction: -1/3 <= z < 0
	float z = (m - 1.0f) / (m + 1.0f);
	float z2 = z * z;

	// ln(m) ~= 2 * (z + z^3/3 + z^5/5 + ... + z^9/9)
	float p = 0.11111111111f;
	p = 0.14285714285f + z2 * p;
	p = 0.20000000000f + z2 * p;
	p = 0.33333333333f + z2 * p;

	float ln_m = 2.0f * z * (1.0f + z2 * p);

	return ln_m + (float)exp * 0.69314718f;

}

NTC_THERMISTOR_ADCStatusTypeDef NTC_THERMISTOR_ReadADCValue(NTC_THERMISTOR_HandleTypeDef *device, ADC_HandleTypeDef *hadc) {

	if (device == NULL || hadc == NULL) {
		return ADC_ERROR;
	}

	// 1. Trigger the ADC

	if (HAL_ADC_Start(hadc) != HAL_OK) {
		return ADC_ERROR;
	}

	// 2. Wait for conversion to finish, using a 10ms timeout
	// TODO: Validate what this timeout is on RAD and match it
	HAL_StatusTypeDef hal_status = HAL_ADC_PollForConversion(hadc, 10);

	// 3. If we get HAL_OK, save the ADC value into adc_raw

	if (hal_status == HAL_OK) {
		device->adc_raw = HAL_ADC_GetValue(hadc);
	}

	// 4. Stop the ADC
	HAL_ADC_Stop(hadc);

	// Returning

	if (hal_status == HAL_OK) {
		return ADC_OK;
	} else if (hal_status == HAL_TIMEOUT) {
		return ADC_TIMEOUT;
	} else if (hal_status == HAL_BUSY) {
		return ADC_BUSY;
	} else {
		return ADC_ERROR;
	}

}

NTC_THERMISTOR_StatusTypeDef NTC_THERMISTOR_GetResistance(NTC_THERMISTOR_HandleTypeDef *device) {

	// Check for division by 0

	// adc_max - device->adc_raw rearranged

	if (ADC_MAX <= device->adc_raw || device->adc_raw == 0) {
		return NTC_THERMISTOR_ERROR;
	}

	// Check for 0

	device->resistance_thermistor = device->fixed_resistance * ((float)device->adc_raw / (ADC_MAX - (float)device->adc_raw));

	if (device->resistance_thermistor <= 0) {
		return NTC_THERMISTOR_ERROR;
	} else {
		return NTC_THERMISTOR_OK;
	}

}


NTC_THERMISTOR_StatusTypeDef NTC_THERMISTOR_GetTempCelcius(NTC_THERMISTOR_HandleTypeDef *device) {

	if (device->resistance_25c == 0.0f || device->resistance_thermistor == 0.0f) {
		return NTC_THERMISTOR_ERROR;
	}
	float ln_argument = device->resistance_thermistor / device->resistance_25c;
	float inverse_temperature = (1.0f / ROOM_TEMPERATURE_KELVIN) + ((fast_ln(ln_argument) / device->beta));
	device->temp = (1.0f / inverse_temperature) - KELVIN_CELCIUS_CONVERSION;

	return NTC_THERMISTOR_OK;
}

NTC_THERMISTOR_StatusTypeDef NTC_THERMISTOR_OrchestratorFunction(NTC_THERMISTOR_HandleTypeDef *device, ADC_HandleTypeDef *hadc) {
	// 1. Read the ADC

	NTC_THERMISTOR_ADCStatusTypeDef adc_status = NTC_THERMISTOR_ReadADCValue(device, hadc);

	if (adc_status != ADC_OK) {
		if (adc_status == ADC_ERROR) {
			return NTC_THERMISTOR_ERROR;
		} else if (adc_status == ADC_BUSY) {
			return NTC_THERMISTOR_BUSY;
		} else if (adc_status == ADC_TIMEOUT) {
			return NTC_THERMISTOR_TIMEOUT;
		}
	}


	// 2. Get the resistance
	// This one only returns NTC_THERMISTOR_ERROR so we are fine to cheat it like this

	if (NTC_THERMISTOR_GetResistance(device) != NTC_THERMISTOR_OK) {
		return NTC_THERMISTOR_ERROR;
	}

	// 3. Get the temperature in Celcius

	if (NTC_THERMISTOR_GetTempCelcius(device) != NTC_THERMISTOR_OK) {
		return NTC_THERMISTOR_ERROR;
	}

	// good return

	return NTC_THERMISTOR_OK;
}





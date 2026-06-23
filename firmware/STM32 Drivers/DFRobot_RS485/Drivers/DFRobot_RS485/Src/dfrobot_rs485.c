/*
 * dfrobot_rs485.c
 *
 *  Created on: May 27, 2026
 *      Author: gobind
 */


#include "dfrobot_rs485.h"


DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_Init(DFROBOT_RS485_HandleTypeDef *device, UART_HandleTypeDef *huart) {

    if (huart == NULL || HAL_UART_GetState(huart) != HAL_UART_STATE_READY) {
        return DFROBOT_RS485_ERROR;
    }

    if (device == NULL) {
        return DFROBOT_RS485_ERROR;
    }

    device->__huart = huart;

    device->temp_offset = 0.0f;
    device->ph_offset = 0.0f;
    device->humidity_offset = 0.0f;
    device->ec_offset = 0.0f;

    device->temp_scaling = 1.0f;
    device->ph_scaling = 1.0f;
    device->humidity_scaling = 1.0f;
    device->ec_scaling = 1.0f;

    device->temp = 0.0f;
    device->ph = 0.0f;
    device->humidity = 0.0f;
    device->ec = 0;

    device->data_length = 13;

    return DFROBOT_RS485_OK;

}

DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_SetOffset(DFROBOT_RS485_HandleTypeDef *device, DFROBOT_RS485_OffsetSelectionTypeDef offset_type, float value) {

    switch (offset_type) {
        case DFROBOT_RS485_TEMPERATURE_OFFSET:
            device->temp_offset = value;
            break;
        case DFROBOT_RS485_PH_OFFSET:
            device->ph_offset = value;
            break;
        case DFROBOT_RS485_HUMIDITY_OFFSET:
            device->humidity_offset = value;
            break;
        case DFROBOT_RS485_EC_OFFSET:
            device->ec_offset = (uint16_t) value;
            break;
        default:
            return DFROBOT_RS485_ERROR;
    }
    return DFROBOT_RS485_OK;
}

DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_SetScaling(DFROBOT_RS485_HandleTypeDef *device, DFROBOT_RS485_ScaleSelectionTypeDef scale_type, float value) {

    switch (scale_type) {
        case DFROBOT_RS485_TEMPERATURE_SCALE:
            device->temp_scaling = value;
            break;
        case DFROBOT_RS485_PH_SCALE:
            device->ph_scaling = value;
            break;
        case DFROBOT_RS485_HUMIDITY_SCALE:
            device->humidity_scaling = value;
            break;
        case DFROBOT_RS485_EC_SCALE:
            device->ec_scaling = value;
            break;
        default:
            return DFROBOT_RS485_ERROR;
    }
    return DFROBOT_RS485_OK;
}

DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_CRC16_2(DFROBOT_RS485_HandleTypeDef *device) {

    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < 11; i++) {
        crc ^= device->buf[i];

        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    device->crc16_result = ((crc & 0x00FF) << 8) | ((crc & 0xFF00) >> 8);

    // Checking the checksum

    uint16_t received_crc = device->buf[11] << 8 | device->buf[12];

    if (device->crc16_result == received_crc) {
        return DFROBOT_RS485_OK;
    } else {
        return DFROBOT_RS485_ERROR;
    }
}

DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_ParseSensorData(DFROBOT_RS485_HandleTypeDef *device) {

    if (DFROBOT_RS485_CRC16_2(device) != DFROBOT_RS485_OK) {
        return DFROBOT_RS485_ERROR;
    }

    // Information verified, two parts to this
    // One get the original data
    // Two apply the scaling and offsets

    device->humidity = (float)(device->buf[3] << 8 | device->buf[4]) / 10.0f;
    device->temp = (float)(device->buf[5] << 8 | device->buf[6]) / 10.0f;
    device->ec = (uint16_t)(device->buf[7] << 8 | device->buf[8]);
    device->ph = (float)(device->buf[9] << 8 | device->buf[10]) / 10.0f;

    // Offset time

    device->temp = device->temp * device->temp_scaling + device->temp_offset;
    device->ph = device->ph * device->ph_scaling + device->ph_offset;
    device->humidity = device->humidity * device->humidity_scaling + device->humidity_offset;
    device->ec = device->ec * device->ec_scaling + device->ec_offset;

    return DFROBOT_RS485_OK;

}

DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_SensorRead(DFROBOT_RS485_HandleTypeDef *device) {

    if (device == NULL || device->__huart == NULL) {
        return DFROBOT_RS485_ERROR;
    }

    // No nulls, proceed with reading from the sensor

    uint8_t command[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09};

    if (HAL_UART_Transmit(device->__huart, command, 8, 100) != HAL_OK) {
        return DFROBOT_RS485_ERROR;
    }


    // Run a HAL_Delay for processing

    HAL_Delay(10); // todo refine

    // Now run the receive function

    if (HAL_UART_Receive(device->__huart, device->buf, 13, 100) != HAL_OK) {
        return DFROBOT_RS485_ERROR;
    }

    // If successful, parse the information

    if (DFROBOT_RS485_ParseSensorData(device) != DFROBOT_RS485_OK) {
        return DFROBOT_RS485_ERROR;
    }

    return DFROBOT_RS485_OK;




}

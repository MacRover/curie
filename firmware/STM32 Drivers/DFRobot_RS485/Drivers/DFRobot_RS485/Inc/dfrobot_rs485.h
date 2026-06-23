/*
 * dfrobot_rs485.h
 *
 *  Created on: May 27, 2026
 *      Author: gobind
 */

#ifndef DFROBOT_RS485_INC_DFROBOT_RS485_H_
#define DFROBOT_RS485_INC_DFROBOT_RS485_H_

/*

- StatusTypeDef: the status of the driver - enum
- HandleTypeDef: All the parameters and variables

Adam to ask if it has an EEPROM chip because we will need that







*/

typedef enum {
    DFROBOT_RS485_TEMPERATURE_SCALE,
    DFROBOT_RS485_PH_SCALE,
    DFROBOT_RS485_HUMIDITY_SCALE,
    DFROBOT_RS485_EC_SCALE
} DFROBOT_RS485_ScaleSelectionTypeDef;

typedef enum {
    DFROBOT_RS485_TEMPERATURE_OFFSET,
    DFROBOT_RS485_PH_OFFSET,
    DFROBOT_RS485_HUMIDITY_OFFSET,
    DFROBOT_RS485_EC_OFFSET
} DFROBOT_RS485_OffsetSelectionTypeDef;

typedef enum {
    DFROBOT_RS485_OK,
    DFROBOT_RS485_ERROR,
    DFROBOT_RS485_BUSY,
    DFROBOT_RS485_TIMEOUT
} DFROBOT_RS485_StatusTypeDef;

typedef struct {

    float temp_offset;
    float ph_offset;
    float humidity_offset;
    uint16_t ec_offset;

    float temp_scaling;
    float ph_scaling;
    float humidity_scaling;
    uint16_t ec_scaling;

    float temp;
    float ph;
    float humidity;
    uint16_t ec;

    uint8_t data_length;
    uint16_t crc16_result;

    uint8_t buf[13];

    UART_HandleTypeDef *__huart;

} DFROBOT_RS485_HandleTypeDef;

/* FUNCTIONS


    gurt
    hello to whoever is reading this (manan looking over gobinds shoulder rn 🥀)

- Init
- Offset
- Scaling
- crc16_2
- parse_sensor_data
- Sensor_read
    
*/

DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_Init(DFROBOT_RS485_HandleTypeDef *device, UART_HandleTypeDef *huart);
DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_SetOffset(DFROBOT_RS485_HandleTypeDef *device, DFROBOT_RS485_OffsetSelectionTypeDef offset_type);
DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_SetScaling(DFROBOT_RS485_HandleTypeDef *device, DFROBOT_RS485_ScaleSelectionTypeDef scale_type);
DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_CRC16_2(DFROBOT_RS485_HandleTypeDef *device);
DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_ParseSensorData(DFROBOT_RS485_HandleTypeDef *device);
DFROBOT_RS485_StatusTypeDef DFROBOT_RS485_SensorRead(DFROBOT_RS485_HandleTypeDef *device);

#endif /* DFROBOT_RS485_INC_DFROBOT_RS485_H_ */
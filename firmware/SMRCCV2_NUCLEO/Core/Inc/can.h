/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

#define CAN_MESSAGE_IDENTIFIER_SMRCCV2 0x03 // TODO CHECK WITH JOHN
#define CAN_MESSAGE_IDENTIFIER_GLOBAL 0x00
#define CAN_MESSAGE_IDENTIFIER_MASK 0b1111
#define CAN_MESSAGE_IDENTIFIER_OFFSET 25

#define CAN_MESSAGE_RESPONSE_SMRCCV2 0x03 // TODO CHECK WITH JOHN
#define CAN_MESSAGE_RESPONSE_OFFSET 18

#define CAN_MESSAGE_COMMAND_MASK 0xFF
#define CAN_MESSAGE_COMMAND_OFFSET 8

#define CAN_MESSAGE_DEVICE_ID_MASK 0xFF
#define CAN_MESSAGE_DEVICE_ID_OFFSET 0

#define CAN_MESSAGE_TIMEOUT_MS 1000

// TODO this is stolen from RAD.  Idk if we need watchdog

typedef struct
{
    CAN_HandleTypeDef hcan;
    uint8_t id;

    uint32_t timer;
    uint8_t watchdog_kick;

    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    CAN_FilterTypeDef canfilterconfig;
    CAN_FilterTypeDef canfilter_global1;
    CAN_FilterTypeDef canfilter_global2;
} SMRCCV2_CAN_TypeDef;

// TODO think on main state a bit more.  I think what we want to do is just do a can mailbox (same as RAD)
// If we receive a CAN message, just flash the LED on board
// If we get send CAN message, just echo back.  More CAN messages will definitely follow

typedef enum
{

    SEND_CAN_MESSAGE = 0x01,
    RECEIVE_CAN_MESSAGE = 0x02

} SMRCCV2_CAN_CommandId;

typedef struct
{
    SMRCCV2_CAN_CommandId command_id;
    uint8_t *data;
    uint8_t dlc;
} SMRCCV2_CAN_Message_TypeDef;

extern SMRCCV2_CAN_TypeDef smrccv2_can;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

void MX_CAN_UpdateIdAndFilters(SMRCCV2_CAN_TypeDef *smrccv2_can_handle);

void MX_CAN_Broadcast_Odometry_Message(SMRCCV2_CAN_TypeDef *smrccv2_can_handle, SMRCCV2_STATUS_TypeDef status);

void MX_CAN_Broadcast_Health_Message(SMRCCV2_CAN_TypeDef *smrccv2_can_handle, SMRCCV2_STATUS_TypeDef status);

void MX_CAN_Broadcast_Double_Data(SMRCCV2_CAN_TypeDef *smrccv2_can_handle, double value, uint16_t message_id);
void MX_CAN_Broadcast_Uint32_Data(SMRCCV2_CAN_TypeDef *smrccv2_can_handle, uint32_t value, uint16_t message_id);
void MX_CAN_Broadcast_Uint16_Data(SMRCCV2_CAN_TypeDef *smrccv2_can_handle, uint16_t value, uint16_t message_id);
void MX_CAN_Broadcast_Uint8_Data(SMRCCV2_CAN_TypeDef *smrccv2_can_handle, uint8_t value, uint16_t message_id);

void MX_CAN_Broadcast_SMRCCV2_Status(SMRCCV2_CAN_TypeDef *smrccv2_can_handle,
        SMRCCV2_STATUS_TypeDef status);

uint32_t __encode_ext_can_id(uint8_t device_id, uint8_t message_id);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_RS485_Pin GPIO_PIN_2
#define USART2_TX_RS485_GPIO_Port GPIOA
#define USART2_RX_RS485_Pin GPIO_PIN_3
#define USART2_RX_RS485_GPIO_Port GPIOA
#define PUMP1_OUT_Pin GPIO_PIN_5
#define PUMP1_OUT_GPIO_Port GPIOA
#define PUMP2_OUT_Pin GPIO_PIN_6
#define PUMP2_OUT_GPIO_Port GPIOA
#define HEATER_OUT_Pin GPIO_PIN_7
#define HEATER_OUT_GPIO_Port GPIOA
#define TEMP_IN_Pin GPIO_PIN_0
#define TEMP_IN_GPIO_Port GPIOB
#define I2C2_SCL_CLS_Pin GPIO_PIN_10
#define I2C2_SCL_CLS_GPIO_Port GPIOB
#define I2C2_SDA_CLS_Pin GPIO_PIN_11
#define I2C2_SDA_CLS_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_15
#define LED_RED_GPIO_Port GPIOB
#define DFROBOT_DIR_Pin GPIO_PIN_8
#define DFROBOT_DIR_GPIO_Port GPIOA
#define I2C1_SCL_EEPROM_Pin GPIO_PIN_6
#define I2C1_SCL_EEPROM_GPIO_Port GPIOB
#define I2C1_SDA_EEPROM_Pin GPIO_PIN_7
#define I2C1_SDA_EEPROM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

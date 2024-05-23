/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "SX1278.h"
#include "RTE_FlashingPort.h"
#include "GW_Config.h"
#include "UserInterface.h"
#include "Flashing_LoRa.h"
#include "Encrypt_if.h"
extern IWDG_HandleTypeDef hiwdg;

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

extern SX1278_hw_t SX1278_hw_1;

extern SX1278_t SX1278_1;

extern SX1278_hw_t SX1278_hw_2;

extern SX1278_t SX1278_2;

extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2;

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
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define RESET_Pin GPIO_PIN_0
#define RESET_GPIO_Port GPIOB
#define DIO_Pin GPIO_PIN_1
#define DIO_GPIO_Port GPIOB
#define MODE_Pin GPIO_PIN_2
#define MODE_GPIO_Port GPIOB
#define DIO_2_Pin GPIO_PIN_10
#define DIO_2_GPIO_Port GPIOB
#define RESET2_Pin GPIO_PIN_11
#define RESET2_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define RECEIIVE_Pin GPIO_PIN_8
#define RECEIIVE_GPIO_Port GPIOB
#define SEND_Pin GPIO_PIN_9
#define SEND_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

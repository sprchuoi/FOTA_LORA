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
#include "SX1278_hw.h"
#include "sht3x.h"

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
#define DIO0_Pin GPIO_PIN_1
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI1_IRQn
#define Mode_Pin GPIO_PIN_2
#define Mode_GPIO_Port GPIOB
#define Receive_Pin GPIO_PIN_8
#define Receive_GPIO_Port GPIOB
#define Send_Pin GPIO_PIN_9
#define Send_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern SX1278_hw_t SX1278_hw;
extern SX1278_t SX1278;
extern IWDG_HandleTypeDef hiwdg;
//extern DHT_Name dht11;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
#define TEMP 0x01U
#define ADC 0x0U
#define GAS  0x03U
#define CO   0x04U
#define CO2  0x05U
#define LIGH 0x06U
#define PRESSURE 0x07U
#define ALITUDE  0x08U
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

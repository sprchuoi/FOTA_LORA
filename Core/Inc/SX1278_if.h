/**
  ******************************************************************************
  * @file    SX1278_if.h
  * @author  Binh Nguyen
  * @brief   This file contains HAL common defines, enumeration, macros and
  *          structures definitions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Sprchuoi.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __SX1278_IF_H__
#define __SX1278_IF_H__
#include "SX1278.h"
#include "main.h"
#define NULL_DATA 0
typedef enum{
	LORA_OKE = 0,
	LORA_BUSSY = 1,
	LORA_ERROR = 2,
	LORA_TIMEOUT = 3,
	LORA_FLASHING = 0x32
} LoRa_Return_t;

LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module , uint8_t* buffer,uint8_t addr ,uint8_t ACK_req);
uint8_t LORA_IF_GetData_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length );
LoRa_Return_t LORA_IF_Stransmit_Data_Frame(SX1278_t *module, uint8_t *txBuffer, uint8_t length, uint32_t timeout);
#endif /*__SX1278_IF_H__*/

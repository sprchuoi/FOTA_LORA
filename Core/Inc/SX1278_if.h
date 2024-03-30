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
#include "operation.h"
#define NULL_DATA 0
#define MAX_TRY_REQ 10
#define MAX_TIME_OUT 2000
#define DATA_LENGTH_FW 128
typedef enum{
	LORA_OKE = 0x0,
	LORA_BUSSY = 0x1,
	LORA_ERROR = 0x2,
	LORA_TIMEOUT = 0x3,
	LORA_FLASHING = 32
} LoRa_Return_t;

extern uint8_t buffer_req[8];
extern uint8_t buffer_packet[132];
extern uint8_t buffer_flashing_data[128];
extern uint8_t ret ;
extern uint8_t buffer_resp[8];
extern uint8_t packet_lost;
extern uint8_t ret;
extern uint8_t counter;
LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module , uint8_t* buffer_req,uint8_t* buffer_resp ,
		uint8_t ret , uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp);
LoRa_Return_t LORA_IF_Stransmit_Fragment_Firmware(SX1278_t *module , uint8_t* buffer_packet ,uint8_t* buffer_flashing_data,
		uint8_t* buffer_resp, uint8_t addr ,uint8_t no , uint8_t ACK_resp);
uint8_t LORA_IF_GetData_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length );
LoRa_Return_t LORA_IF_Stransmit_Data_Frame(SX1278_t *module, uint8_t *txBuffer, uint8_t length, uint32_t timeout);
LoRa_Return_t  LORA_IF_GetFragment_Firmware(SX1278_t *module , uint8_t* buffer_packet ,uint8_t* buffer_flashing_data ,
		uint8_t* buffer_resp ,uint8_t addr , uint8_t no , uint8_t ACK_resp );
LoRa_Return_t LORA_IF_Stransmit_Response(SX1278_t *module , uint8_t *buffer_req , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp);
#endif /*__SX1278_IF_H__*/

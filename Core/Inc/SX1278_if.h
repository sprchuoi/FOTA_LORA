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
#include "Encrypt_if.h"
#define NULL_DATA 				0u
#define MAX_TRY_REQ 			3000u
#define MAX_TIME_OUT			2000u
#define MAX_TIME_OUT_RECEIVE	3000u
#define TIME_DELAY 				1000u
#define SIZE_BUFFER_8BYTES  	8u
#define SIZE_BUFFER_16BYTES		16u
#define SIZE_BUFFER_128BYTES  	128u
#define SIZE_BUFFER_80BYTES		80u
#define SIZE_BUFFER_112BYTES	112u

#define WAIT_PACKET_SEND 		00u
typedef enum{
	LORA_OKE = 0x0,
	LORA_BUSSY = 0x1,
	LORA_ERROR = 0x2,
	LORA_TIMEOUT = 0x3,
	LORA_FLASHING_ERROR = 0x32
} LoRa_Return_t;

extern uint8_t buffer_req[16];
extern uint8_t buffer_packet[80];
extern uint8_t buffer_packet_Rx[112];
extern uint8_t *buffer_flashing_data;
extern uint8_t ret ;
extern uint8_t packet_lost;
extern uint8_t buffer_resp[16];
extern uint8_t no_packet ;
extern uint8_t Chanel;
extern uint8_t Power;
extern uint8_t SF;
extern uint8_t BandWidth;
extern uint8_t CR;
extern uint8_t u8buffer_Pack_Lost[88];
extern uint8_t buffer_req_2[16];
LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module , uint8_t* buffer_req,uint8_t* buffer_resp ,
		uint8_t ret ,uint8_t ACK_req , uint8_t ACK_resp );
LoRa_Return_t LORA_IF_Stransmit_Fragment_Firmware(SX1278_t *module ,uint8_t* buffer_flashing_data);
uint8_t LORA_IF_GetData_Frame(SX1278_t *module ,uint32_t unicast_address, uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length , uint8_t ACK_resp);
LoRa_Return_t LORA_IF_Stransmit_Data_Frame(SX1278_t *module, uint8_t *txBuffer, uint8_t length, uint32_t timeout);
uint8_t LORA_IF_GetFragment_Firmware(SX1278_t *module , uint8_t* buffer ,uint8_t no ,  uint8_t ret , uint32_t timeout , uint8_t length);
uint8_t LORA_IF_GetData_End_Frame(SX1278_t *module, uint8_t *rxBuffer, uint32_t unicast_addr  , uint8_t length, uint32_t timeout);
#endif /*__SX1278_IF_H__*/

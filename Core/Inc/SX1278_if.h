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
#define NULL_DATA 			0
#define MAX_TRY_REQ 		10
#define MAX_TIME_OUT		2000
#define DATA_LENGTH_FW 		128
#define TIME_DELAY 			1000
#define SIZE_BUFFER_8BYTES  8
#define SIZE_BUFFER_16BYTES 16
#define SIZE_BUFFER_132BYTES  132
typedef enum{
	LORA_OKE = 				0x0,
	LORA_BUSSY = 			0x1,
	LORA_ERROR = 			0x2,
	LORA_TIMEOUT = 			0x3,
	LORA_FLASHING = 		32
} LoRa_Return_t;


//Address define
#define ADDR_UNICAST 0x01
#define ADDR_NODE_1  0x01
#define ADD_BROADCAST 0xFF
// Sequence flash code define

#define GW_START_OTA	                				0x01
#define MCU_ENTER_FLASHMODE                         	0X02
#define FL_FRAGMENT_FIRMWARE							0xFE
#define GW_SYNC_CONFIG									0x20
#define MCU_RECEIVED_CONFIG								0x21
#define MCU_ENTER_FBL									0x12
#define GW_PROVIDE_FW_INFO								0x30
#define MCU_RECEIVED_SIZE_CODE							0x31
#define MCU_REQUEST_PACKET 								0x32
#define GW_START_SEND_FW								0x33
#define MCU_RECEIVE_SUCCESS         				    0x34
#define MCU_CHECK_SECTOR_FINISHING                  	0x35
#define MCU_ACCEPT_RECEIVING_PACKET_OF_CODE     		0x36
#define MCU_ACKNOWLEDGE_LINE_OF_CODE_RECEIVED   		0x37
#define MCU_WRITE_SUCCESS								0x38
#define GW_ACKNOWLEDGE_FINISHING_SENDING_CODE  		    0x39
#define MCU_ACKNOWLEDGE_FINISHING               		0x77
#define MCU_ACKNOWLEDGE_ACTIVE_CODE_CORRECT     		0x78
#define MCU_ACKNOWLEDGE_ACTIVE_CODE_NOT_CORRECT 		0x79
#define MCU_ACKNOWLEDGE_BACKUP_CODE_CORRECT     		0x7A
#define MCU_ACKNOWLEDGE_BACKUP_CODE_NOT_CORRECT 		0x7B
#define GW_ACKNOWLEDGE_FINISHING                        0x7C
#define GW_REQ_PARAMETER								0x8C


extern uint32_t Local_u32SizeOfCode;
extern uint8_t  Local_u8index_fragment;
extern uint32_t u32Buffer_Flash[16];
extern uint8_t buffer_req[16];
extern uint8_t buffer_packet[132];
extern uint8_t buffer_flashing_data[128];
extern uint8_t ret ;
extern uint8_t buffer_resp[16];
extern uint8_t packet_lost;
extern uint8_t counter;
extern uint32_t BL_CRCCheck;
LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module , uint8_t* buffer_req,uint8_t* buffer_resp ,
		uint8_t ret , uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp);
LoRa_Return_t LORA_IF_Stransmit_Request_Finish(SX1278_t *module , uint8_t *buffer_req , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp);
LoRa_Return_t LORA_IF_Stransmit_Fragment_Firmware(SX1278_t *module , uint8_t* buffer_packet ,uint8_t* buffer_flashing_data,
		uint8_t* buffer_resp, uint8_t addr ,uint8_t no , uint8_t ACK_resp);
uint8_t LORA_IF_GetData_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length );
LoRa_Return_t LORA_IF_Stransmit_Data_Frame(SX1278_t *module, uint8_t *txBuffer, uint8_t length, uint32_t timeout);
LoRa_Return_t LORA_IF_Stransmit_Response(SX1278_t *module , uint8_t *buffer_req , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp);
LoRa_Return_t LORA_IF_Stransmit_Response_Finish(SX1278_t *module , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req);
uint8_t  LORA_IF_GetFragment_Firmware(SX1278_t *module , uint8_t* buffer_packet ,uint8_t* buffer_flashing_data,
		uint8_t addr);
LoRa_Return_t LORA_IF_Stransmit_Response_Flashing(SX1278_t *module ,uint8_t* buffer_resp ,
		uint8_t no, uint8_t ret, uint8_t addr ,uint8_t ACK_resp);
void LORA_IF_Init(void);
void LORA_IF_RECIEVE(void);
#endif /*__SX1278_IF_H__*/

/*
 * RTE_EncryptPort.h
 *
 *  Created on: Apr 27, 2024
 *      Author: quang
 */

#ifndef INC_API_ENCRYPTPORT_H_
#define INC_API_ENCRYPTPORT_H_
#include "Rte.h"
#include "Receive_FW_UART.h"

#define SIZE_BUFFER_DATA 0x7FU // 128 bytes
/**************************************************************************/
/*                         Runnable entities                              */
/**************************************************************************/
// 						Define Port Flashing SYSTEM STATE
/**************************************************************************/
/*                         Ports Write SystemState                         */
/**************************************************************************/
#define RTE_RUNNABLE_SYSTEM_STATE_WriteData		Rte_SystemState_WriteData
/**************************************************************************/
/*                         Ports Read SystemState                        */
/**************************************************************************/
#define RTE_RUNNABLE_SYSTEM_STATE_ReadData		Rte_SystemState_ReadData
// 						Define Port Encrypt
/**************************************************************************/
/*                         Ports Write Encrypt                            */
/**************************************************************************/
#define RTE_RUNNABLE_ENCRYPT_DATA_BUFFER_WriteData  Rte_EncyptedBuffer_WriteData
#define RTE_RUNNABLE_ENCRYPT_DATA_BUFFER_FLAG_WriteData Rte_EncyptedBufferFlag_WriteData
/**************************************************************************/
/*                         Ports Read Encrypt                             */
/**************************************************************************/

#define RTE_RUNNABLE_ENCRYPT_DATA_BUFFER_ReadData  Rte_EncyptedBuffer_ReadData
#define RTE_RUNNABLE_ENCRYPT_DATA_BUFFER_FLAG_ReadData Rte_EncyptedBufferFlag_ReadData



//						Define Port Update FW
/**************************************************************************/
/*                         Ports Write Update FW                         */
/**************************************************************************/
#define    RTE_RUNNABLE_CRC_VALUE_WriteData          Rte_Crc_WriteData
#define    RTE_RUNNABLE_APP_VER_WriteData			 Rte_ApplVer_WriteData
#define    RTE_RUNNABLE_CODE_SIZE_WriteData          Rte_CodeSize_WriteData
#define    RTE_RUNNABLE_NODE_ADDR_WriteData          Rte_NodeAddr_WriteData
#define    RTE_RUNNABLE_HEADER_ACK_FLAG_WriteData    Rte_HeaderAckFlag_WriteData
#define    RTE_RUNNABLE_DOWNLOAD_PROGRESS_WriteData  Rte_UpdateProgress_WriteData
/**************************************************************************/
/*                         Ports Read Receive Update FW                   */
/**************************************************************************/

#define    RTE_RUNNABLE_CRC_VALUE_ReadData           Rte_Crc_ReadData
#define    RTE_RUNNABLE_APP_VER_ReadData			 Rte_ApplVer_ReadData
#define    RTE_RUNNABLE_CODE_SIZE_ReadData           Rte_CodeSize_ReadData
#define    RTE_RUNNABLE_NODE_ADDR_ReadData           Rte_NodeAddr_ReadData
#define    RTE_RUNNABLE_HEADER_ACK_FLAG_ReadData   	 Rte_HeaderAckFlag_ReadData
#define    RTE_RUNNABLE_DOWNLOAD_PROGRESS_ReadData   Rte_UpdateProgress_ReadData


//					 Define Port UserInterface
/**************************************************************************/
/*                        Ports  Write UI						 		  */
/**************************************************************************/
#define     RTE_RUNNABLE_USER_RESPONSE_WriteData            Rte_UserResponse_WriteData
#define 	RTE_RUNNABLE_UI_ERROR_WriteData 				Rte_UI_Error_WriteData

/**************************************************************************/
/*                        Ports  Read UI						 		  */
/**************************************************************************/
#define     RTE_RUNNABLE_USER_RESPONSE_ReadData            Rte_UserResponse_ReadData
#define 	RTE_RUNNABLE_UI_ERROR_ReadData 				Rte_UI_Error_ReadData

/**************************************************************************/
/*                         Ports Write Sending Packet Update FW           */
/**************************************************************************/
#define    RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData		Rte_PacketSendLoraNum_WriteData
#define    RTE_RUNNABLE_FLAG_LORA_RESP_WriteData			Rte_FlagLoRaResp_WriteData
#define    RTE_RUNNABLE_CONFIG_LORA_WriteData				Rte_FlagConfigLoRA_WriteData
/**************************************************************************/
/*                         Ports Read Sending Packet Update FW            */
/**************************************************************************/
#define    RTE_RUNNABLE_PACKET_SEND_LORA_NUM_ReadData		Rte_PacketSendLoraNum_ReadData
#define    RTE_RUNNABLE_FLAG_LORA_RESP_ReadData				Rte_FlagLoRaResp_ReadData
#define    RTE_RUNNABLE_CONFIG_LORA_ReadData				Rte_FlagConfigLoRA_ReadData
/**************************************************************************/
/*                         Ports Read Sending Packet LoRa            */
/**************************************************************************/
#define    RTE_RUNNABLE_PACKET_LORA_REIVECED_WriteData		Rte_PacketLoRaReceived_WriteData

/**************************************************************************/
/*                         Ports Read Sending Packet LoRa            */
/**************************************************************************/
#define    RTE_RUNNABLE_PACKET_LORA_REIVECED_ReadData		Rte_PacketLoRaReceived_ReadData
#endif /* INC_API_ENCRYPTPORT_H_ */


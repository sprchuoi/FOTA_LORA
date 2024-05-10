/*
 * RTE_FlashingPort.c
 *
 *  Created on: Apr 27, 2024
 *      Author: quang
 */


#include <Flash_If.h>
#include "RTE_FlashingPort.h"



/**************************************************************************/
/*                         Global Variable                                */
/**************************************************************************/
// Information of IMG FW
static uint8_t gl_u8NodeAddr;
static uint8_t gl_u8TypeFlag;
static uint32_t gl_u32ImgSize;
static uint8_t gl_u8AppVersion;
static uint32_t gl_u32CRCValue;
static uint32_t gl_u32ConfigLoRa;
// Variable for using Uart Interrupt received
static ReceiveFWUartType gl_RXUartInternal_State;
static uint32_t gl_u32ReceiveUart;
static uint32_t gl_u32remain_byteflash;
static uint8_t gl_u8RXBuffer[PACKET_1024bytes];
static uint8_t gl_u8RXBuffer_Header[HEADER_CONFIG_SIZE];
static uint8_t gl_u8RXBuffer_Flag_Req_Bytes;
static uint8_t gl_u8NumberPacket_Uart;
static uint16_t gl_u16NumberPacket_LoRa;
static uint32_t gl_u32Remain_Byte;
static uint8_t gl_u8DownLoadUpdateProgress;
static uint8_t gl_u8DonwLoadPercentProogess;
static uint32_t gl_u32ReceiveBytes;
static uint8_t gl_u8RxUserResp;
// System State
static uint8_t gl_u8SystemState;

/**************************************************************************/
/*                         Module Functions                               */
/**************************************************************************/

void ReceiveFWUpdate_Init(void){
	gl_u8NodeAddr 						= INITIAL_VALUE_ZERO;
	gl_u8TypeFlag 						= INITIAL_VALUE_ZERO;
	gl_u32ImgSize 						= INITIAL_VALUE_ZERO;
	gl_u8AppVersion 					= INITIAL_VALUE_ZERO;
	gl_u32CRCValue 						= INITIAL_VALUE_ZERO;
	gl_u32ConfigLoRa 					= INITIAL_VALUE_ZERO;
	gl_u8SystemState 					= SYS_IDLE;
	gl_RXUartInternal_State 			= RX_IDLE;
	gl_u8RxUserResp						= INITIAL_VALUE_ZERO;
	gl_u8DonwLoadPercentProogess 		= INITIAL_VALUE_ZERO;
	gl_u32Remain_Byte					= INITIAL_VALUE_ZERO;
	gl_u8DownLoadUpdateProgress 		= INITIAL_VALUE_ZERO;
	gl_u8NumberPacket_Uart 				= INITIAL_VALUE_ZERO;
	gl_u16NumberPacket_LoRa 				= INITIAL_VALUE_ZERO;
	gl_u32ReceiveBytes 					= INITIAL_VALUE_ZERO;
	HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer_Header, 1);
	F_voidInitVariables();
}
void ReceiveFWUpdate_MainFunc(void){
	switch (gl_RXUartInternal_State) {
		case RX_IDLE:
		{
			uint8_t retVal = RTE_RUNNABLE_USER_RESPONSE_ReadData(&gl_u8RxUserResp);
			if(RTE_E_OKE == retVal){
				if(gl_u8RxUserResp == ACCEPT_UPDATE){
					gl_RXUartInternal_State = RX_ACCEPT_UPDATE;
				}
				else if(gl_u8RxUserResp == REFUSE_UPDATE){
					gl_RXUartInternal_State = RX_DENY_UPDATE;
				}
				else{
					/* Update State normal in hear*/
				}
			}
			break;
		}
		case RX_ACCEPT_UPDATE :
		{
			// Request ESP send Update
			gl_u8RXBuffer_Flag_Req_Bytes = NEW_UPDATE_REQUEST_ACCEPT;
			HAL_UART_Transmit(&huart2, gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			// Erase APPLU address to Update
			F_Erase_Image(IMAGE_NEW_FIRMWARE);
			gl_RXUartInternal_State = RX_RECEIVED_PACKET;
			break;
		}
		case RX_DENY_UPDATE:
		{
			// Request ESP send Update
			gl_u8RXBuffer_Flag_Req_Bytes = NEW_UPDATE_REQUEST_DENY;
			HAL_UART_Transmit(&huart2, gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			// Erase APPLU address to Update
			gl_u8RxUserResp = INITIAL_VALUE_ZERO;
			RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_IDLE);
			// Enable Uart Interrupt hear
			__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
			gl_RXUartInternal_State = RX_IDLE;
			break;
		}
		case RX_RECEIVED_HEADER:
		{
			/* Get Request ESP send header */
			gl_u8RXBuffer_Flag_Req_Bytes = ESP_SEND_HEADER_FLAG;
			HAL_UART_Transmit(&huart2, gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			/*Receive Header file*/
			HAL_UART_Receive(&huart2, gl_u8RXBuffer_Header, HEADER_CONFIG_SIZE, HAL_MAX_DELAY);
			/*Put Information FW to RTE*/
			gl_u32CRCValue =(gl_u8RXBuffer_Header[15] >> SHIFT_24_BIT) | (gl_u8RXBuffer_Header[14]  >>SHIFT_16_BIT)
						   |(gl_u8RXBuffer_Header[13]>>  SHIFT_8_BIT) | (gl_u8RXBuffer_Header[12] >> SHIFT_0_BIT);
			gl_u32ImgSize =(gl_u8RXBuffer_Header[7]>>  SHIFT_24_BIT) | (gl_u8RXBuffer_Header[6] >> SHIFT_16_BIT)
						  |(gl_u8RXBuffer_Header[5]>>  SHIFT_8_BIT) | (gl_u8RXBuffer_Header[4] >> SHIFT_0_BIT) ;
			gl_u8AppVersion = (gl_u8RXBuffer_Header[9]>>  SHIFT_8_BIT) | (gl_u8RXBuffer_Header[8] >> SHIFT_0_BIT);
			gl_u32ConfigLoRa =(gl_u8RXBuffer_Header[19]>>  SHIFT_24_BIT) | (gl_u8RXBuffer_Header[18] >> SHIFT_16_BIT)
							 |(gl_u8RXBuffer_Header[17]>>  SHIFT_8_BIT)  | (gl_u8RXBuffer_Header[16] >> SHIFT_0_BIT);
			//Update Parameter
			F_FlashWordToAddress(FLAG_PARAMETER_GW_CONFIG, gl_u32ConfigLoRa);
			gl_u8NodeAddr =(gl_u8RXBuffer_Header[0]>>  SHIFT_8_BIT) | (gl_u8RXBuffer_Header[1] >> SHIFT_0_BIT);
			gl_u8RXBuffer_Flag_Req_Bytes = (gl_u8RXBuffer_Header[3]>>  SHIFT_0_BIT);
			if(gl_u8RXBuffer_Flag_Req_Bytes == HEADER_FLAG_FW_INFO ){
				RTE_RUNNABLE_APP_VER_WriteData(gl_u8AppVersion);
				RTE_RUNNABLE_CODE_SIZE_WriteData(gl_u32ImgSize);
				RTE_RUNNABLE_CRC_VALUE_WriteData(gl_u32CRCValue);
				RTE_RUNNABLE_NODE_ADDR_WriteData(gl_u8NodeAddr);
				RTE_RUNNABLE_CONFIG_LORA_WriteData(gl_u32ConfigLoRa);
				gl_RXUartInternal_State = RX_RECEIVED_HEADER;
				//Calculate Number packet Rx Uart
				gl_u8NumberPacket_Uart = (uint8_t)gl_u32ImgSize/PACKET_1024bytes;
				RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData(gl_u16NumberPacket_LoRa);
				gl_u8RXBuffer_Flag_Req_Bytes = HEADER_FLAG_RECEIVED;
				HAL_UART_Transmit(&huart2, gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
				/*Change State System*/
				RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_NEW_UPDATE_REQ);
				gl_RXUartInternal_State = RX_IDLE;
			}
			else{
				/*Invalid Request*/
				gl_u8RXBuffer_Flag_Req_Bytes = INVALID_REQUEST;
				HAL_UART_Transmit(&huart2, gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			}
			break;
		}
		case RX_RECEIVED_PACKET:
		{
			gl_u8RXBuffer_Flag_Req_Bytes = ESP_SEND_NEXT_PACKET;
			HAL_UART_Transmit(&huart2, gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			if(gl_u8NumberPacket_Uart > 0)
			{
				HAL_UART_Receive(&huart2, gl_u8RXBuffer, PACKET_1024bytes, HAL_MAX_DELAY);
				gl_u8NumberPacket_Uart--;
				//Flash to block
				F_FlashBlockToAddress(gl_u8RXBuffer, gl_u32Remain_Byte);
				gl_u8RXBuffer_Flag_Req_Bytes = GATEWAY_ACCEPT_PACKET;
				HAL_UART_Transmit(&huart2, gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
				gl_u32ReceiveBytes += PACKET_1024bytes;
			}
			else if((gl_u8NumberPacket_Uart == 0)&&(gl_u32Remain_Byte > 0))
			{
				HAL_UART_Receive(&huart2, gl_u8RXBuffer, PACKET_1024bytes, HAL_MAX_DELAY);
				F_FlashBlockToAddress(gl_u8RXBuffer, gl_u32Remain_Byte);
				gl_u8RXBuffer_Flag_Req_Bytes = RX_DONE_INSTALL_FW;
				gl_u32ReceiveBytes += gl_u32Remain_Byte;

			}
			else{
				/*ERROR*/
			}
			/*Calculate Progress*/
			gl_u8DonwLoadPercentProogess = (float)gl_u32ReceiveBytes /(float)gl_u32ImgSize;
			gl_u8DownLoadUpdateProgress = gl_u8DonwLoadPercentProogess*100;
			/*Write to RTE */
			RTE_RUNNABLE_DOWNLOAD_PROGRESS_WriteData(gl_u8DownLoadUpdateProgress);


			break;
		}

		case RX_END_STATE:
		{
			RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_ENCRYPT_FW);
			__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
			gl_u8NodeAddr = INITIAL_VALUE_ZERO;
			gl_u8TypeFlag = INITIAL_VALUE_ZERO;
			gl_u32ImgSize =INITIAL_VALUE_ZERO;
			gl_u8AppVersion = INITIAL_VALUE_ZERO;
			gl_u32CRCValue = INITIAL_VALUE_ZERO ;
			gl_u8SystemState = SYS_IDLE;
			gl_RXUartInternal_State = RX_IDLE;
			gl_u8RxUserResp=INITIAL_VALUE_ZERO;
			gl_u8DonwLoadPercentProogess =INITIAL_VALUE_ZERO;
			gl_u32Remain_Byte= INITIAL_VALUE_ZERO;
			gl_u8DownLoadUpdateProgress = INITIAL_VALUE_ZERO;
			gl_u8NumberPacket_Uart = INITIAL_VALUE_ZERO;
			gl_u16NumberPacket_LoRa = INITIAL_VALUE_ZERO;
			gl_u32ReceiveBytes = INITIAL_VALUE_ZERO;
			RTE_RUNNABLE_DOWNLOAD_PROGRESS_WriteData(INITIAL_VALUE_ZERO);
			break;
		}
		default:
			break;
	}
}

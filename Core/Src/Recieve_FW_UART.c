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
static uint32_t gl_u32NodeAddr;
static uint8_t gl_u8TypeFlag;
static uint32_t gl_u32ImgSize;
static uint16_t gl_u16AppVersion;
static uint32_t gl_u32CRCValue;
static uint32_t gl_u32ConfigLoRa;
// Variable for using Uart Interrupt received
static ReceiveFWUartType gl_RXUartInternal_State;
static uint8_t gl_u8RXBuffer[PACKET_1024bytes];
static uint8_t gl_u8RXBuffer_flash[PACKET_1024bytes];
static uint8_t gl_u8RXBuffer_Header[HEADER_CONFIG_SIZE];
static uint8_t gl_u8RXBuffer_Flag_Req_Bytes;
static uint8_t gl_u8NumberPacket_Uart;
static uint16_t gl_u16NumberPacket_LoRa;
static uint32_t gl_u32Remain_Byte;
static uint8_t gl_u8DownLoadUpdateProgress;
static float gl_u8DonwLoadPercentProogess;
static uint32_t gl_u32ReceiveBytes;
static uint8_t gl_u8RxUserResp;
// System State
static uint8_t gl_u8SystemState;

/**************************************************************************/
/*                         Module Functions                               */
/**************************************************************************/

void ReceiveFWUpdate_Init(void){
	gl_u32NodeAddr 						= INITIAL_VALUE_ZERO;
	gl_u8TypeFlag 						= INITIAL_VALUE_ZERO;
	gl_u32ImgSize 						= INITIAL_VALUE_ZERO;
	gl_u16AppVersion 					= INITIAL_VALUE_ZERO;
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
	//__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
	HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer_Header, 	1);

	F_voidInitVariables();
}
void ReceiveFWUpdate_MainFunc(void){
	switch (gl_RXUartInternal_State) {
		case RX_IDLE:

		{	__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
			if(gl_u8RXBuffer_Header[0] == NEW_UPDATE_REQUEST)
				gl_RXUartInternal_State = RX_ACCEPT_UPDATE;
			break;
		}
		case RX_ACCEPT_UPDATE :
		{
			// Request ESP send Update
			gl_u8RXBuffer_Flag_Req_Bytes = NEW_UPDATE_REQUEST_ACCEPT;
			// Stop IT Timer vs EXT
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
			RTE_RUNNABLE_FLAG_LORA_REQUEST_DEVICE_WriteData(0x04);
			F_Erase_Image(IMAGE_NEW_FIRMWARE);
			HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			// Erase APPLU address to Update
			gl_RXUartInternal_State = RX_RECEIVED_HEADER;
			break;
		}
		case RX_DENY_UPDATE:
		{
			// Request ESP send Update
			gl_u8RXBuffer_Flag_Req_Bytes = NEW_UPDATE_REQUEST_DENY;
			HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
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
			//gl_u8RXBuffer_Flag_Req_Bytes = ESP_SEND_HEADER_FLAG;
			//HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			/*Receive Header file*/
			//HAL_UART_Receive(&huart2, gl_u8RXBuffer_Header, HEADER_CONFIG_SIZE, HAL_MAX_DELAY);
			/*Put Information FW to RTE*/
			//byte 7 to byte 4 is size

			gl_u32ImgSize =(gl_u8RXBuffer_Header[8]*1000000) + (gl_u8RXBuffer_Header[7]*10000)+
						  (gl_u8RXBuffer_Header[6]*100) + (gl_u8RXBuffer_Header[5]) ;
			gl_u16AppVersion = (gl_u8RXBuffer_Header[9] << SHIFT_8_BIT) | (gl_u8RXBuffer_Header[10] << SHIFT_0_BIT);
			gl_u32ConfigLoRa =(gl_u8RXBuffer_Header[14] << SHIFT_24_BIT) | (gl_u8RXBuffer_Header[12] << SHIFT_16_BIT)
							 |(gl_u8RXBuffer_Header[11] <<  SHIFT_8_BIT)  | (gl_u8RXBuffer_Header[13] << SHIFT_0_BIT);
			//Update Parameter
			//F_FlashWordToAddress(FLAG_PARAMETER_GW_CONFIG, gl_u32ConfigLoRa);



			gl_u32NodeAddr =(gl_u8RXBuffer_Header[0]<<  SHIFT_24_BIT)|(gl_u8RXBuffer_Header[1]<<  SHIFT_16_BIT)
					|(gl_u8RXBuffer_Header[2]<<  SHIFT_8_BIT)|(gl_u8RXBuffer_Header[3]<<  SHIFT_0_BIT);
			//CRC firmware
			gl_u32CRCValue =(gl_u8RXBuffer_Header[17]<<  SHIFT_24_BIT)|(gl_u8RXBuffer_Header[16]<<  SHIFT_16_BIT)
							|(gl_u8RXBuffer_Header[15]<<  SHIFT_8_BIT)|(gl_u8RXBuffer_Header[14]<<  SHIFT_0_BIT);


			gl_u8RXBuffer_Flag_Req_Bytes = gl_u8RXBuffer_Header[4];
			if(gl_u8RXBuffer_Flag_Req_Bytes == ESP_SEND_HEADER_FLAG ){
				GW_voidEraseRestoreConfigPage(FLAG_STATUS_BANKSECOND_APP_VER_ADDRESS,(uint32_t)gl_u16AppVersion);
				GW_voidEraseRestoreConfigPage(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS,gl_u32ImgSize);
				GW_voidEraseRestoreConfigPage(FLAG_STATUS_ADDRESS_TARGET_ADDRESS,gl_u32NodeAddr);
				GW_voidEraseRestoreConfigPage(FLAG_PARAMETER_GW_CONFIG,gl_u32ConfigLoRa);
				GW_voidEraseRestoreConfigPage(FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS, gl_u32CRCValue);
				GW_voidEraseRestoreConfigPage(FLAG_STATUS_GW_CONFIG,GW_CONFIG_PARAMETER_SET);
				RTE_RUNNABLE_APP_VER_WriteData(gl_u16AppVersion);
				RTE_RUNNABLE_CODE_SIZE_WriteData(gl_u32ImgSize);
				//RTE_RUNNABLE_CRC_VALUE_WriteData(gl_u32CRCValue);
				RTE_RUNNABLE_NODE_ADDR_WriteData(gl_u32NodeAddr);
				RTE_RUNNABLE_CONFIG_LORA_WriteData(gl_u32ConfigLoRa);
				gl_RXUartInternal_State = RX_RECEIVED_HEADER;
				//Calculate Number packet Rx Uart
				gl_u8NumberPacket_Uart = (uint8_t)(gl_u32ImgSize/PACKET_1024bytes)+1;
				/* Get info from header */

				gl_u32Remain_Byte = (gl_u32ImgSize % PACKET_1024bytes);

				RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData(gl_u16NumberPacket_LoRa);
				gl_u8RXBuffer_Flag_Req_Bytes = HEADER_FLAG_RECEIVED;
				//GW_State_Save_State((uint32_t)SYS_IDLE);
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,0);
				HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
				/*Change state */
				RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_RECEIVE_UPDATE);
				gl_RXUartInternal_State = RX_RECEIVED_PACKET;

				__HAL_UART_DISABLE_IT(&huart2 , UART_IT_RXNE);
			}
			else{
				__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
				/*Invalid Request*/
				if (gl_u8RXBuffer_Flag_Req_Bytes == NEW_UPDATE_REQUEST_ACCEPT){
					HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
					memset(gl_u8RXBuffer_Header , 0x00 , 16);
				}

			}

			break;
		}
		case RX_RECEIVED_PACKET:
		{

			if(gl_u32ReceiveBytes == 0)
			{
				HAL_TIM_Base_Start(&htim1);
				gl_u8RXBuffer_Flag_Req_Bytes = ESP_SEND_NEXT_PACKET;
				HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);

				__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);

				gl_u32ReceiveBytes += PACKET_1024bytes;
			}
			else if(gl_u8NumberPacket_Uart >= 1 && gl_u8RXBuffer_Flag_Req_Bytes == MASTER_ACCEPT_PACKET)
			{
				//HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer, PACKET_1024bytes, HAL_MAX_DELAY);
				//HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer, PACKET_1024bytes);
				HAL_TIM_Base_Stop_IT(&htim4);
				gl_u8NumberPacket_Uart--;
				/*Calculate Progress*/
				gl_u8DonwLoadPercentProogess = (float)gl_u32ReceiveBytes /(float)gl_u32ImgSize;
				gl_u8DownLoadUpdateProgress = (uint8_t)(gl_u8DonwLoadPercentProogess*100);
				/*Write to RTE */

				RTE_RUNNABLE_DOWNLOAD_PROGRESS_WriteData(gl_u8DownLoadUpdateProgress);
				//Flash to block
				HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,1);
				__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
				gl_u8RXBuffer_Flag_Req_Bytes = ESP_SEND_NEXT_PACKET;
				gl_u32ReceiveBytes += PACKET_1024bytes;
				//reset counter
				TIM4->CNT = 0;
				HAL_TIM_Base_Start_IT(&htim4);
			}
			if(gl_u8NumberPacket_Uart == 0 && gl_u8RXBuffer_Flag_Req_Bytes == MASTER_RECEIVE_ALL)
			{
				//gl_u8RXBuffer_Flag_Req_Bytes = MASTER_RECEIVE_ALL;
				/*ERROR*/
				HAL_TIM_Base_Stop_IT(&htim4);
				RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_REQUEST_OTA);
				HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
				//F_FlashBlockToAddress(gl_u8RXBuffer, gl_u32Remain_Byte);
				gl_u32ReceiveBytes += gl_u32Remain_Byte;
				gl_u8DonwLoadPercentProogess = (float)gl_u32ReceiveBytes /(float)gl_u32ImgSize;
				gl_u8DownLoadUpdateProgress = (uint8_t)(gl_u8DonwLoadPercentProogess*100);
				RTE_RUNNABLE_DOWNLOAD_PROGRESS_WriteData(gl_u8DownLoadUpdateProgress);
				gl_RXUartInternal_State = RX_END_STATE;
				//GW_State_Save_State((uint32_t)SYS_REQUEST_OTA);
				__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
				HAL_TIM_Base_Stop(&htim1);
			}
			else{
				/*ERROR*/

			}

			break;
		}
		case RX_END_STATE:
		{
			__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
			gl_u8RXBuffer_Flag_Req_Bytes = DONE_OTA;
			HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
			gl_u32NodeAddr = INITIAL_VALUE_ZERO;
			gl_u8TypeFlag = INITIAL_VALUE_ZERO;
			gl_u32ImgSize =INITIAL_VALUE_ZERO;
			gl_u16AppVersion = INITIAL_VALUE_ZERO;
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
/********************HAL_UART_CALLBACK***********/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	//  HAL_GPIO_WritePin(GPIOA	, GPIO_PIN_10,0);
	// for testing time encrypt

	Std_ReturnType retVal;
 	retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_u8SystemState);
	// Handle for Get Wifi
	if(gl_u8RXBuffer_Header[0] == ESP_RESET_SPI && gl_u8SystemState != SYS_REQUEST_OTA){
		if(RTE_E_OKE == retVal){
			RTE_RUNNABLE_SYSTEM_STATE_WriteData(WAIT_FOR_ESP_CONNECT);
			__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
			HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer_Header, 1);
		}
	}
	else if( gl_u8RXBuffer_Header[0] == WIFI_CONNECTED){
		if(RTE_E_OKE == retVal){
			RTE_RUNNABLE_SYSTEM_STATE_WriteData(WIFI_CONNECTED);
			__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
			gl_RXUartInternal_State  = RX_IDLE;
			HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer_Header, 1);
		}
	}
	else if(gl_u8RXBuffer_Header[0] == NEW_UPDATE_REQUEST && gl_u8SystemState != SYS_REQUEST_OTA ){

		if(RTE_E_OKE == retVal){
			//
			// Change state to system update
			RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_NEW_UPDATE_REQ);
			gl_RXUartInternal_State = RX_ACCEPT_UPDATE;
			// Disable interrupt UART
			__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
			HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer_Header, 20);
		}
		else{
			/* Refuse the update request as the system is not ready to receive updates */
			gl_u8RXBuffer_Flag_Req_Bytes = GATEWAY_BUSY;
			//HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);

		}
	}
	else if (gl_u8RXBuffer_Flag_Req_Bytes == ESP_SEND_NEXT_PACKET || gl_u8RXBuffer_Flag_Req_Bytes == MASTER_ACCEPT_PACKET ){
		// Enable IT timer to make reset
		HAL_TIM_Base_Start_IT(&htim4);
		__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);

		Decrypt_MainFunc((uint8_t*)gl_u8RXBuffer);

		if(gl_u8NumberPacket_Uart == 1){
			HAL_TIM_Base_Stop_IT(&htim4);
			F_FlashBlockToAddress(gl_u8RXBuffer, gl_u32Remain_Byte);
			HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer_Header, 1);
			gl_u8RXBuffer_Flag_Req_Bytes = MASTER_RECEIVE_ALL;
			gl_u8NumberPacket_Uart--;
			//RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_REQUEST_OTA);
		}
		else{
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
			F_FlashBlockToAddress(gl_u8RXBuffer, PACKET_1024bytes);
			gl_u8RXBuffer_Flag_Req_Bytes = MASTER_ACCEPT_PACKET;
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
		}
		HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer, PACKET_1024bytes);
		memset(gl_u8RXBuffer , 0xff , 1024);

		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);



	}
	else if(gl_RXUartInternal_State == RX_RECEIVED_HEADER)
	{
		HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer, PACKET_1024bytes);
	}
	else
	{
		/**/
		gl_u8RXBuffer_Flag_Req_Bytes = NEW_UPDATE_REQUEST_ACCEPT;;
		//HAL_UART_Transmit(&huart2, &gl_u8RXBuffer_Flag_Req_Bytes, 1, HAL_MAX_DELAY);
		__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);
		HAL_UART_Receive_IT(&huart2, gl_u8RXBuffer_Header, 1);
	}

	//__HAL_UART_ENABLE_IT(&huart2 , UART_IT_RXNE);


}



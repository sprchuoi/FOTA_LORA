/*
 * LORA_IF.c
 *
 *  Created on: Mar 25, 2024
 *      Author: quang
 */

#include "SX1278_if.h"
#include "SX1278.h"


struct AES_ctx ctx;
static uint8_t AES_CBC_128_Key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
static uint8_t AES_CBC_128_IV[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
uint8_t LORA_IF_GetFragment_Firmware(SX1278_t *module , uint8_t* buffer , uint8_t no , uint8_t ret , uint32_t timeout , uint8_t length){
	ret = SX1278_LoRaEntryRx(module, length, timeout);
		HAL_Delay(100);
		ret = SX1278_LoRaRxPacket(module);
		if ( ret > 0 ) {
			ret = SX1278_read(module, (uint8_t*) buffer, ret);
			if(buffer[0] == ADDR_MASTER && buffer[2] == FL_FRAGMENT_FIRMWARE){
				no = buffer[1];
				return FL_FRAGMENT_FIRMWARE ;
			}
		}
	    return 0;
}
uint8_t LORA_IF_GetData_Frame(SX1278_t *module , uint8_t* buffer_resp , uint8_t ret , uint32_t timeout , uint8_t length , uint8_t ACK_resp ){
	uint32 local_u32timeout = 0;
	ret = SX1278_LoRaEntryRx(module, length, timeout);
	while(1){
		ret = SX1278_LoRaRxPacket(module);
		if ( ret > 0 ) {
			// Replace Receive Led hear
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			ret = SX1278_read(module, (uint8_t*) buffer_resp, ret);
			AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
			AES_CTR_xcrypt_buffer(&ctx, (uint8_t*) buffer_resp, 16);
			if(buffer_resp[1] == ADDR_SLAVE_NODE_1 && buffer_resp[0] == ADDR_UNICAST && buffer_resp[2] == ACK_resp)
				return 1;
			/*Received but wrong request*/

		}
		HAL_Delay(1);
		local_u32timeout++;
		if(local_u32timeout == timeout)
			return 0;
		/*Lost Packet Hear*/
	}
}
//LoRa_Return_t LORA_IF_Stransmit_Data_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length ){
//    ret = SX1278_LoRaEntryTx(module, length , timeout);
//	ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer, length, timeout);
//	return LORA_OKE;
//}
LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module , uint8_t *buffer_req , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp ){
	buffer_req[1] = addr;
	buffer_req[2] = ACK_req;
	AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
	AES_CTR_xcrypt_buffer(&ctx, (uint8_t*) buffer_req, 16);
	//init to TX mode
	 ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	 ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_req, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	/*Read the first Frame */

	 if(ret >0){
		//Replace Blink Send hear
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		return LORA_OKE;
	 }
	 return LORA_TIMEOUT;
}

LoRa_Return_t LORA_IF_Stransmit_Fragment_Firmware(SX1278_t *module ,uint8_t* buffer_flashing_data ){
	uint8_t counter = 0;
	while(1){
	 ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_80BYTES, MAX_TIME_OUT);
	 ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_flashing_data, SIZE_BUFFER_80BYTES, MAX_TIME_OUT);
	 if(ret){
		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		 HAL_Delay(WAIT_PACKET_SEND);
		/*Read the first Frame */
//		switch(local_u8Check_Code = LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_resp , ret , MAX_TIME_OUT , SIZE_BUFFER_16BYTES)){
//			case(MCU_ACKNOWLEDGE_FINISHING):
//				buffer_packet[2] = buffer_resp[2];
		 return LORA_OKE ;
//			case(MCU_IMAGE_CRC_NOT_CORRECT):
//				return LORA_FLASHING_ERROR;
	 }
//		}
	 else{
		 counter++;
		 if(counter == MAX_TRY_REQ){
			 return LORA_ERROR;
		 }
	 }

	}
}

uint8_t LORA_IF_GetData_End_Frame(SX1278_t *module, uint8_t *rxBuffer, uint8_t length, uint32_t timeout){
	uint32 local_u32timeout = 0;
	uint8_t local_u8addrNode = 0;
	ret = SX1278_LoRaEntryRx(module, length, timeout);
	AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
	while(1){
		ret = SX1278_LoRaRxPacket(module);
		RTE_RUNNABLE_NODE_ADDR_ReadData(&local_u8addrNode);
		if ( ret > 0 ) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			ret = SX1278_read(module, (uint8_t*) rxBuffer, ret);
			AES_CTR_xcrypt_buffer(&ctx, (uint8_t*) rxBuffer, length);
			if(rxBuffer[1] == local_u8addrNode)
				return rxBuffer[2];
		}
		local_u32timeout++;
		HAL_Delay(1);
		if(local_u32timeout == MAX_TIME_OUT_RECEIVE )
			return 0;
	}
}

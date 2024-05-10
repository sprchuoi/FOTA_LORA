/*
 * LORA_IF.c
 *
 *  Created on: Mar 25, 2024
 *      Author: quang
 */

#include "SX1278_if.h"
#include "SX1278.h"
#include "BL_Program.h"
uint8_t counter ;
static uint8_t AES_CBC_128_Key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
static uint8_t AES_CBC_128_IV[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
struct AES_ctx ctx;
uint16_t  LORA_IF_GetFragment_Firmware(SX1278_t *module , uint8_t* buffer_packet ,uint8_t* buffer_flashing_data ,
		uint8_t addr){
	counter = 0 ;
	uint16_t index_fragmemt = 0;
	//clear data buffer
	//clearDataBuffer((uint8_t*) buffer_packet,132);
	/* Configuration LoRa to Receive firmware*/
	ret = SX1278_LoRaEntryRx(module, SIZE_BUFFER_80BYTES , MAX_TIME_OUT);
	while(1){
		ret = SX1278_LoRaRxPacket(module);
		if ( ret > 0 ) {
			ret = SX1278_read(module, (uint8_t*) buffer_packet, ret);
			AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
			AES_CTR_xcrypt_buffer(&ctx, (uint8_t*) buffer_packet, SIZE_BUFFER_80BYTES);
			if(buffer_packet[0] == ADDR_BOARDCAST  && buffer_packet[1] == addr  && buffer_packet[2] == FL_FRAGMENT_FIRMWARE){
				/* Copy data from buffer packet to buffer flashing data*/
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				index_fragmemt =(buffer_packet[3]<<SHIFT_8_BIT)|(buffer_packet[4] <<SHIFT_0_BIT);
				return  index_fragmemt;
			}
			else if ( buffer_packet[0] == ADDR_BOARDCAST  && buffer_packet[1] == addr  && buffer_packet[2] == GW_ACKNOWLEDGE_FINISHING_SENDING_CODE ){
				return  GW_SEND_DONE;
			}
			else{
				counter++;
				HAL_Delay(1);
				if(counter == MAX_TIME_OUT){
				// If not receive return 0
					return 0;
				}
			}
		}
	}

}


uint8_t LORA_IF_TransferData_Frame(SX1278_t *module , uint8_t* buffer_req , uint8_t ret , uint32_t timeout , uint8_t length , uint8_t ACK_req)
{
	buffer_req[0] = ADDR_UNICAST;
	buffer_req[1] = ADDR_NODE_1;
	buffer_req[2] = ACK_req;
	AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
	AES_CTR_xcrypt_buffer(&ctx, (uint8_t*) buffer_req, 16);
	ret = SX1278_LoRaEntryTx(module, length, timeout);
	ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_req, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
		if (ret) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			return 1;
		}
		else{
			return 0;
		}
}

LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module  , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr  , uint8_t ACK_resp){
	counter = 0;
	ret = SX1278_LoRaRxPacket(module);
	if(ret > 0){
		/*Read the first Frame
		 *counter to retry connect until get resp signal
		 */
		 ret = SX1278_read(module, (uint8_t*) buffer_resp, ret);
		 AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
		 AES_CTR_xcrypt_buffer(&ctx,(uint8_t*)buffer_resp, 16);
		 if(buffer_resp[0]== ADDR_BOARDCAST  && buffer_resp[1] == addr && buffer_resp[2]  == ACK_resp){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			// Response packet
			return LORA_OKE ;
		 }
	 }
	return LORA_BUSSY;
}

LoRa_Return_t LORA_IF_Stransmit_Response_Flashing(SX1278_t *module ,uint8_t* TxBuffer
	, uint8_t ret, uint8_t addr ,uint8_t ACK_resp){
	TxBuffer[0] = ADDR_UNICAST;
	TxBuffer[1] = addr;
	TxBuffer[2] = ACK_resp;
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//AES_CTR_xcrypt_buffer(&ctx,(uint8_t*)buffer_resp, 128);
	ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_128BYTES  , MAX_TIME_OUT);
	AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
	AES_CTR_xcrypt_buffer(&ctx,(uint8_t*)TxBuffer, SIZE_BUFFER_128BYTES);
	ret = SX1278_LoRaTxPacket(module, (uint8_t*) TxBuffer, SIZE_BUFFER_128BYTES, MAX_TIME_OUT);
	if(ret){
		// Toggle pin led to notify response
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		return LORA_OKE;

	}
	return LORA_ERROR;
}

LoRa_Return_t LORA_IF_Stransmit_Fragment_Firmware(SX1278_t *module , uint8_t* buffer_packet ,uint8_t* buffer_flashing_data ,
		uint8_t* buffer_resp ,uint8_t addr , uint8_t no , uint8_t ACK_resp ){
	uint8_t counter = 0;
	/*Store addr to buffer */
	buffer_packet[0] = 0xFF ; //Broadcast addr
	buffer_packet[1] = addr ; // the  addr
	buffer_packet[2] = no;
	buffer_packet[3] = FL_FRAGMENT_FIRMWARE;
	// Copy array buffer_data to buffer flashing
	copy_Array_BL(buffer_packet , buffer_flashing_data, 112);
	while(1){
	 ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_128BYTES  , MAX_TIME_OUT);
	 ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_packet, SIZE_BUFFER_128BYTES, MAX_TIME_OUT);
	 if(ret){
		/*Read the first Frame */
		if(LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_resp , ret , MAX_TIME_OUT , SIZE_BUFFER_16BYTES ) == ACK_resp){
			return LORA_OKE ;
		}
	 }
	 counter++;
	 if(counter == MAX_TRY_REQ){
		 return LORA_ERROR;
	 }
	}
}


LoRa_Return_t LORA_IF_Stransmit_Response(SX1278_t *module , uint8_t* buffer_resp , uint8_t ret , uint8_t addr , uint8_t ACK_resp){
	buffer_resp[0]= ADDR_UNICAST;
	buffer_resp[1] = addr;
	buffer_resp[2]= ACK_resp;
	ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_16BYTES  , MAX_TIME_OUT);
	AES_init_ctx_iv(&ctx, AES_CBC_128_Key, AES_CBC_128_IV);
	AES_CTR_xcrypt_buffer(&ctx,(uint8_t*)buffer_resp, SIZE_BUFFER_16BYTES);
	ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_resp, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	if(ret){
		// Toggle pin led to notify response
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		//HAL_Delay(1000);
		return LORA_OKE;
	}
	return LORA_ERROR;
}

/*
 * LORA_IF.c
 *
 *  Created on: Mar 25, 2024
 *      Author: quang
 */

#include "SX1278_if.h"
#include "SX1278.h"
#include "function.h"
#include "UserInterface.h"
#include "aes.h"
uint8_t counter;
uint8_t gl_Status_Flag;
uint8_t ret;
uint8_t buffer_resp[16];
uint8_t buffer_send[16];
uint8_t buffer_DHT[4];
uint16_t MQ2_Val;
uint32_t Node_Address;
uint32_t Address_Receive;
struct AES_ctx ctx;
static uint8_t AES_CTR_128_Key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
static uint8_t AES_CTR_128_IV[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
void LORA_IF_Init(){
	  SX1278_hw.dio0.port = DIO0_GPIO_Port;
	  SX1278_hw.dio0.pin = DIO0_Pin;
	  SX1278_hw.nss.port = NSS_GPIO_Port;
	  SX1278_hw.nss.pin = NSS_Pin;
	  SX1278_hw.reset.port = RESET_GPIO_Port;
	  SX1278_hw.reset.pin = RESET_Pin;
	  SX1278_hw.spi = &hspi1;
	  /* USER CODE END 2 */
	  SX1278.hw = &SX1278_hw;
	  SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, SX1278_LORA_SF_12,
			  SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 16);
	  ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
	  UI_DisplayInformation();
	  Node_Address = FUNC_ReaddataAddress(FLAG_NODE_ID);
	  LORA_IF_RECIEVE();
//	  // Check Sensor Init Read;
//	  FUNC_get_DHT_val((uint8_t* ) buffer_DHT);
//	  MQ2_Val = FUNC_get_MQ_val();
}
void LORA_IF_RECIEVE(){
	while(1){
        FUNC_get_DHT_val((uint8_t *)buffer_DHT);

		UI_Display_DataValue((uint8_t*) buffer_DHT , MQ2_Val);
		uint32_t local_u32addrNode_req = 0;
		if(Address_Receive == Node_Address){
			// reset data
			Address_Receive = 0;
			gl_Status_Flag = buffer_resp[4];
			// Test Jump to FBL
			switch (gl_Status_Flag) {
				case GW_START_OTA:
					FUNC_EraseAndRestore_Header_Page(BL_FLASHING_STATE , BL_BRANCHING_FLAG_SET);
					__disable_irq();    // disable all interrupt
					UI_InitBoot();
					break;
				case GW_REQ_PARAMETER:
					buffer_send[0] = (Node_Address>>SHIFT_24_BIT)&0xff;
					buffer_send[1] = (Node_Address>>SHIFT_16_BIT)&0xff;
					buffer_send[2] = (Node_Address>>SHIFT_8_BIT)&0xff;
					buffer_send[3] = (Node_Address>>SHIFT_0_BIT)&0xff;
					buffer_send[4] = TEMP;
					buffer_send[5] = buffer_DHT[0];
					buffer_send[6] = buffer_DHT[1];
					buffer_send[7] = TEMP;
					buffer_send[8] = buffer_DHT[2];
					buffer_send[9] = buffer_DHT[3];
					buffer_send[10] = GAS;
					buffer_send[11] = 0;
					buffer_send[12] = 0>>SHIFT_8_BIT;
					//HAL_TIM_Base_Start_IT(&htim2);
					//Start Timer
					HAL_NVIC_DisableIRQ(EXTI1_IRQn);
					LORA_IF_SEND((uint8_t*)buffer_send);
					// change to mode Receives
					ret = SX1278_LoRaEntryRx(&SX1278, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
					break;
				default:
					break;
			}
		}
	}
}

void LORA_IF_SEND(uint8_t *buffer_send){
	AES_init_ctx_iv(&ctx, AES_CTR_128_Key, AES_CTR_128_IV);
	AES_CTR_xcrypt_buffer(&ctx, (uint8_t*) buffer_send, 16);
	//init to TX mode
	ret = SX1278_LoRaEntryTx(&SX1278, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) buffer_send, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	if(ret){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	}

	__HAL_GPIO_EXTI_CLEAR_IT(DIO0_Pin);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	// Reset Buffer to default
	memset((uint8_t*)buffer_resp , 0xff , 16);
	//ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint32_t address_break = 0;
	if(GPIO_Pin == DIO0_Pin){
			ret = SX1278_LoRaRxPacket(&SX1278);
			if(ret > 0 ){
				ret = SX1278_read(&SX1278, (uint8_t*) buffer_resp, ret);
				AES_init_ctx_iv(&ctx, AES_CTR_128_Key, AES_CTR_128_IV);
				AES_CTR_xcrypt_buffer(&ctx, (uint8_t*) buffer_resp, 16);
				Address_Receive = (buffer_resp[0]<<SHIFT_24_BIT)|(buffer_resp[1]<<SHIFT_16_BIT)
													|(buffer_resp[2]<<SHIFT_8_BIT)|(buffer_resp[3]<<SHIFT_0_BIT);
				if (Address_Receive == Node_Address){
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
					HAL_NVIC_DisableIRQ(EXTI1_IRQn);
					__HAL_GPIO_EXTI_CLEAR_IT(DIO0_Pin);
				}
			}

	}
}



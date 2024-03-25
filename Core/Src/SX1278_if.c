/*
 * LORA_IF.c
 *
 *  Created on: Mar 25, 2024
 *      Author: quang
 */

#include "SX1278_if.h"
#include "SX1278.h"

char buffer_req[16] = "";
#define ADDR_MASTER  0x123
uint8_t LORA_IF_GetData_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length ){
    ret = SX1278_LoRaEntryRx(module, length, timeout);
	HAL_Delay(100);
	ret = SX1278_LoRaRxPacket(module);
	if ( ret > 0 ) {
		ret = SX1278_read(module, (uint8_t*) buffer, ret);
		if(buffer[0] == ADDR_MASTER)
			return buffer[1];
	}
    return 0;
}
//LoRa_Return_t LORA_IF_Stransmit_Data_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length ){
//    ret = SX1278_LoRaEntryTx(module, length , timeout);
//	ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer, length, timeout);
//	return LORA_OKE;
//}
LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module , uint8_t *buffer, uint8_t addr ,uint8_t ACK_req){
	buffer_req[0] = addr;
	buffer_req[1] = ACK_req;
	while(1){
	 ret = SX1278_LoRaEntryTx(module, 16  , 2000);
	 ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer, 16, 2000);
	 if(ret){
		return LORA_OKE;
		/*Read the first Frame */
		if(LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_MOSI , ret , 2000 , 130 ) == LORA_OKE){
			break;
		}
	 }


	}
	return LORA_ERROR;
}


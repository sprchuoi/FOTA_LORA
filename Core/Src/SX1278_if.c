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
uint8_t  LORA_IF_GetFragment_Firmware(SX1278_t *module , uint8_t* buffer_packet ,uint8_t* buffer_flashing_data ,
		uint8_t addr){
	counter = 0 ;
	//clear data buffer
	//clearDataBuffer((uint8_t*) buffer_packet,132);
		/* Configuration LoRa to Receive firmware*/
	while(1){
		ret = SX1278_LoRaEntryRx(module, SIZE_BUFFER_132BYTES , MAX_TIME_OUT);
		HAL_Delay(100);
		ret = SX1278_LoRaRxPacket(module);
		if ( ret > 0 ) {
			ret = SX1278_read(module, (uint8_t*) buffer_packet, ret);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			if(buffer_packet[0] == ADDR_UNICAST  && buffer_packet[1] == addr  && buffer_packet[3] == FL_FRAGMENT_FIRMWARE){
				/* Copy data from buffer packet to buffer flashing data*/

				return buffer_packet[2] ;
			}
			else{
				// If not receive return 0
				return 0;
			}
		}
	}

}


uint8_t LORA_IF_GetData_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length ){
    ret = SX1278_LoRaEntryRx(module, length, timeout);
	HAL_Delay(100);
	ret = SX1278_LoRaRxPacket(module);
	if ( ret > 0 ) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		ret = SX1278_read(module, (uint8_t*) buffer, ret);
		if(buffer[0]== ADDR_BOARDCAST  && buffer[1] == ADDR_NODE_1)
			return buffer[3];
	}
    return 0;
}
//LoRa_Return_t LORA_IF_Stransmit_Data_Frame(SX1278_t *module , uint8_t* buffer , uint8_t ret , uint32_t timeout , uint8_t length ){
//    ret = SX1278_LoRaEntryTx(module, length , timeout);
//	ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer, length, timeout);
//	return LORA_OKE;
//}
LoRa_Return_t LORA_IF_Stransmit_Request(SX1278_t *module , uint8_t *buffer_req , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp){
	counter = 0;
	buffer_req[0] = ADDR_UNICAST;
	buffer_req[1] = addr ;
	buffer_req[3] = ACK_req;
	while(1){
	 ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_8BYTES  , MAX_TIME_OUT);
	 ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_req, SIZE_BUFFER_8BYTES, MAX_TIME_OUT);
	 if(ret){
		/*Read the first Frame
		 *counter to retry connect until get resp signal
		 */
		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		 HAL_Delay(TIME_DELAY);
		if(LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_resp , ret , MAX_TIME_OUT , SIZE_BUFFER_8BYTES ) == ACK_resp){
			return LORA_OKE ;
		}
		else counter++;
	 }
	 if(counter == MAX_TRY_REQ){
		 return LORA_TIMEOUT;
	 }
	}

}


LoRa_Return_t LORA_IF_Stransmit_Request_Finish(SX1278_t *module , uint8_t *buffer_req , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp){
	buffer_req[0] = ADDR_UNICAST;
	buffer_req[1] = addr ;
	buffer_req[3] = ACK_req;
	while(1){
	 ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_8BYTES  , MAX_TIME_OUT);
	 ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_req, SIZE_BUFFER_8BYTES, MAX_TIME_OUT);
	 if(ret){
		/*Read the first Frame
		 *counter to retry connect until get resp signal
		 */
		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		 HAL_Delay(TIME_DELAY);
		if(LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_resp , ret , MAX_TIME_OUT , SIZE_BUFFER_8BYTES ) == ACK_resp){
			return LORA_OKE ;
		}
	 }
	 return LORA_TIMEOUT;
	}
}

LoRa_Return_t LORA_IF_Stransmit_Response(SX1278_t *module , uint8_t *buffer_req , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req , uint8_t ACK_resp){
	uint8_t counter = 0;
	while(1){
		if((LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_resp , ret , MAX_TIME_OUT , SIZE_BUFFER_8BYTES ) == ACK_resp)){
			return LORA_OKE;
		}
		else {
			counter++;
			/* counter to retry connect until get resp signal */
			if(counter == MAX_TRY_REQ){
				if(LORA_IF_Stransmit_Request(module,(uint8_t* ) buffer_req,(uint8_t* ) buffer_resp, ret, addr, ACK_req, ACK_resp) == LORA_TIMEOUT){
					return LORA_TIMEOUT;
				}
			}

		}
	}
}

LoRa_Return_t LORA_IF_Stransmit_Response_Finish(SX1278_t *module , uint8_t* buffer_resp ,
		uint8_t ret, uint8_t addr ,uint8_t ACK_req){
	while(1){
		if((LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_resp , ret , MAX_TIME_OUT , SIZE_BUFFER_8BYTES ) == ACK_req)){
			return LORA_OKE;
		}
		return LORA_TIMEOUT;

	}

}

LoRa_Return_t LORA_IF_Stransmit_Response_Flashing(SX1278_t *module ,uint8_t* buffer_resp ,
		uint8_t no , uint8_t ret, uint8_t addr ,uint8_t ACK_resp){
	buffer_resp[0] = ADDR_UNICAST;
	buffer_resp[1] = addr;
	buffer_resp[2] = no ;
	buffer_resp[3] = ACK_resp;
	ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_8BYTES  , MAX_TIME_OUT);
	ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_resp, SIZE_BUFFER_8BYTES, MAX_TIME_OUT);
	if(ret){
		// Toggle pin led to notify response
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(TIME_DELAY);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
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
	copy_Array_BL(buffer_packet , buffer_flashing_data, 128);
	while(1){
	 ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_132BYTES  , MAX_TIME_OUT);
	 ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_packet, SIZE_BUFFER_132BYTES, MAX_TIME_OUT);
	 if(ret){
		/*Read the first Frame */
		if(LORA_IF_GetData_Frame(module ,(uint8_t*) buffer_resp , ret , MAX_TIME_OUT , SIZE_BUFFER_8BYTES ) == ACK_resp){
			return LORA_OKE ;
		}
	 }
	 counter++;
	 if(counter == MAX_TRY_REQ){
		 return LORA_ERROR;
	 }
	}
}

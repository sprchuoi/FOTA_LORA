/*
 * Flashing_LoRa.c
 *
 *  Created on: Mar 26, 2024
 *      Author: quang
 */
#include  "Flashing_LoRa.h"
#include  "SX1278_if.h"
#include "UserInterface_privateFunc.h"
/***************Read Global Variable****************/
static uint8_t gl_u8StatusFlash 			= 	INITIAL_VALUE_ZERO;
static uint8_t gl_u8Counterloss				=	INITIAL_VALUE_ZERO;
static uint16_t gl_u16No_Packet_Flash		=	INITZERO_START+1;
static uint8_t gl_start_array_backup 		= 	INITIAL_VALUE_ZERO;
static uint32_t gl_u32TargetAddr 				= 	INITIAL_VALUE_ZERO;
static uint32_t gl_u32CRCFW 				= 	INITIAL_VALUE_ZERO;
static uint16_t gl_u16AppVer 	   			=	INITIAL_VALUE_ZERO;
static uint32_t gl_u32CodeSize 				=	INITIAL_VALUE_ZERO;
static uint16_t gl_u16NumberPacket_LoRa       =   INITIAL_VALUE_ZERO;
static uint8_t gl_u8ErrorFlag 				=   INITIAL_VALUE_ZERO;
static uint8_t gl_u8start_array_backup		=   INITIAL_VALUE_ZERO;
static uint8_t gl_u32Config_LoRa_Flag		=	INITIAL_VALUE_ZERO;
static uint8_t gl_u8backup_flag 			=   INITIAL_VALUE_ZERO;
uint16_t lost_counter_pos 					=	INITIAL_VALUE_ZERO;
static uint8_t gl_FlagLoraSendDevice 		= 	INITIAL_VALUE_ZERO;
uint8_t buffer_resp[16];
uint8_t buffer_packet_Rx[112];
uint8_t buffer_packet[80];
uint8_t buffer_req[16];
uint8_t  *buffer_flashing_data;
uint8_t u8buffer_Pack_Lost[88];
uint16_t buffer_number_pack_lost[704];
uint8_t buffer_resp_2[16];
//uint8_t buffer_req_2[16];
/***************Read Global Variable****************/
uint8_t ret = 0 ;
struct AES_ctx ctx_req;
static uint8_t AES_CTR_128_Key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
static uint8_t AES_CTR_128_IV[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
//uint32_t FL_uint32localAddress = ACTIVE_IMAGE + 0x80 ;
FL_Return_t FL_Syns_Config(uint32_t unicast_address, uint8_t* buffer_req, uint8_t* buffer_resp
	, uint8_t SF ,  uint8_t BandWidth , uint8_t CR ){
	/* Add info to packet */
	buffer_req[3] = SF ;
	buffer_req[4] = BandWidth ;
	/*	Wait to get Response from Node */
	buffer_req[5] = CR ;
	if(LORA_IF_Stransmit_Request(&SX1278_2, (uint8_t*) buffer_req,(uint8_t*) buffer_resp ,
			ret,GW_SYNC_CONFIG , MCU_RECEIVED_CONFIG ) == LORA_OKE)
	{
		if(LORA_IF_GetData_Frame(&SX1278_2 ,unicast_address ,(uint8_t*) buffer_resp , ret , 5000 , SIZE_BUFFER_16BYTES ,MCU_RECEIVED_CONFIG ) == 1)
		{
			SX1278_init(&SX1278_1, 434000000, SX1278_POWER_17DBM, SF,
							BandWidth, CR, SX1278_LORA_CRC_EN, 16);
			// Setting LORA For Flashing
			return FL_SUCCESS;
		}
	}
	return FL_FAIL;
}
FL_Return_t Flashing_init(uint8_t address_update, uint8_t* buffer_packet, uint8_t* buffer_resp , uint16_t number_packet )
{
	/* REQUEST START FLASHING */
	buffer_req[11] = number_packet >>SHIFT_8_BIT;
	buffer_req[10] = number_packet >>SHIFT_0_BIT;
	if(LORA_IF_Stransmit_Request(&SX1278_1, (uint8_t*) buffer_packet,(uint8_t*) buffer_resp ,
			ret ,GW_PROVIDE_FW_INFO , MCU_ACCEPT_RECEIVING_PACKET_OF_CODE ) == LORA_OKE){
		return FL_SUCCESS ;
	}
	return FL_FAIL;
}

/* Start send size of FW*/
//FL_Return_t Flashing_size(uint8_t address_update, uint8_t* buffer_packet, uint8_t* buffer_resp , uint32_t number_packet ){
//	/* SEND THE SIZE OF CODE FOR SLAVE TO GET THE ACTUAL SIZE FW */
//	if(LORA_IF_Stransmit_Request(&SX1278, (uint8_t*) buffer_packet,(uint8_t*) buffer_resp  ,
//			ret ,  address_update ,GW_PROVIDE_FW_INFO , MCU_RECEIVED_SIZE_CODE) == LORA_OKE){
//		return FL_SUCCESS ;
//	}
//	return FL_FAIL;
//}
/* Start Send Fragment */
FL_Return_t Sequence_Process(uint8_t* buffer_flashing_data){
	if(LORA_IF_Stransmit_Fragment_Firmware(&SX1278_1,(uint8_t*) buffer_flashing_data  ) == LORA_OKE){
		return FL_SUCCESS;
	}
	return FL_FAIL;
}
/* End Programming process*/
uint8_t Flashing_end(uint8_t broadcast_addr,uint32_t unicast_addr , uint8_t* buffer_packet, uint8_t* buffer_resp ,uint8_t ACK_REQ){
	uint32_t counter=0;
	buffer_packet[0] = broadcast_addr ; // broadcast
	buffer_packet[1] = broadcast_addr ; // broadcast
	buffer_packet[2] = ACK_REQ;
	AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
	ret = SX1278_LoRaEntryTx(&SX1278_1, SIZE_BUFFER_80BYTES, MAX_TIME_OUT);
	AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_packet, SIZE_BUFFER_80BYTES);
	while(1){
		 ret = SX1278_LoRaTxPacket(&SX1278_1, (uint8_t*) buffer_packet, SIZE_BUFFER_80BYTES, MAX_TIME_OUT);
		 if(ret){
			 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			 return LORA_IF_GetData_End_Frame(&SX1278_1, (uint8_t*) buffer_resp, unicast_addr,SIZE_BUFFER_112BYTES, MAX_TIME_OUT);
		 }
		 else{
			 counter++;
			 if(counter == MAX_TRY_REQ)
			 {
				 return -1;
			 }
		}

	}
}

/*request get value from gateway*/
uint8_t Send_request(SX1278_t *module , uint32_t unicast_addr , uint8_t * buffer_req_2 , uint8_t ACK_REQ){
	buffer_req_2[0]  = (unicast_addr>>SHIFT_24_BIT)&0xff;
	buffer_req_2[1]  = (unicast_addr>>SHIFT_16_BIT)&0xff;
	buffer_req_2[2]  = (unicast_addr>>SHIFT_8_BIT)&0xff;
	buffer_req_2[3]  = (unicast_addr>>SHIFT_0_BIT)&0xff;
	buffer_req_2[4]  = GW_REQ_PARAMETER;
	uint32_t counter =0;
	ret = SX1278_LoRaEntryTx(module, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
	AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_req_2, 16);
	ret = SX1278_LoRaTxPacket(module, (uint8_t*) buffer_req_2, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	if(ret){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	}
	ret = SX1278_LoRaEntryRx(module, 16, 3000);
	while(1){
		//Rte_FlagLoRaResp_ReadData(&flag_LoRa);
		uint32_t local_u32addrNode_req = 0;
		ret = SX1278_LoRaRxPacket(module);
		if ( ret > 0 ) {
			ret = SX1278_read(module, (uint8_t*) buffer_resp_2, ret);
			AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
			AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_resp_2, 16);
			//convert buffer to address_node_req
			local_u32addrNode_req = (buffer_resp_2[0] << SHIFT_24_BIT) |(buffer_resp_2[1] << SHIFT_16_BIT)
											 |(buffer_resp_2[2] << SHIFT_8_BIT) | (buffer_resp_2[3] << SHIFT_0_BIT);
			if(local_u32addrNode_req == ADDRESS__MAC_NODE_1 ||local_u32addrNode_req == ADDRESS__MAC_NODE_2 ||
					local_u32addrNode_req == ADDRESS__MAC_NODE_3 )
			{
				HAL_UART_Transmit(&huart2, &buffer_resp_2, 16, HAL_MAX_DELAY);
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
				return 1;
			}
			memset(buffer_resp_2 , 0xff ,16);
		}
		counter++;
		HAL_Delay(1);
		if(counter == 3000){
			return 0;
		}
	}
	//Get data from SPI_2


}
uint8_t Flashing_Request(uint8_t broadcast_addr ,uint8_t addr_node,  uint8_t* buffer_req ,uint8_t ACK_REQ ,uint8_t ACK_RESP ){
	uint16_t counter_tx =0;
	uint32_t Address_update = INIT_VAL_ZERO;
	RTE_RUNNABLE_NODE_ADDR_ReadData(&Address_update);
	buffer_req[0] =  (Address_update>> SHIFT_24_BIT)& 0xFF; // unicast addr
	buffer_req[1] = (Address_update>> SHIFT_16_BIT)& 0xFF ; // unicast addr
	buffer_req[2] = (Address_update>> SHIFT_8_BIT)& 0xFF ;  // unicast addr
	buffer_req[3] = (Address_update>> SHIFT_0_BIT)& 0xFF ;  // unicast addr
	buffer_req[4] = ACK_REQ;	// Not use
	//Get the current Packet;
	RTE_RUNNABLE_PACKET_SEND_LORA_NUM_ReadData(&counter_tx);
	ret = SX1278_LoRaEntryTx(&SX1278_2, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
	AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_req, 16);
	ret = SX1278_LoRaTxPacket(&SX1278_2, (uint8_t*) buffer_req, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	if(ret){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		counter_tx++;
		RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData(counter_tx);
		return 1;
	}
	return -1;
}
uint8_t Wait_Accept_OTA(uint32_t addr_node,  uint8_t* buffer_resp ,uint8_t ACK_RESP ){
	uint32_t counter_rx = INITIAL_VALUE_ZERO;
	uint32_t Node_Address_update = addr_node;
	uint32_t Node_Address_receive = INITIAL_VALUE_ZERO;
	ret = SX1278_LoRaEntryRx(&SX1278_2, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
	while(1){
		ret = SX1278_LoRaRxPacket(&SX1278_2);
		AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
		if(ret > 0){
			ret = SX1278_read(&SX1278_2, (uint8_t*) buffer_resp, ret);
			AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_resp, 16);
			Node_Address_receive = (buffer_resp[0] << SHIFT_24_BIT) |(buffer_resp[1] << SHIFT_16_BIT)
								|(buffer_resp[2] << SHIFT_8_BIT)|(buffer_resp[3] << SHIFT_0_BIT);
			if(Node_Address_receive == Node_Address_update && buffer_resp[4]== ACK_RESP ){
				return 1;
			}

		}
		else{
			counter_rx++;
			//delay 1ms
			HAL_Delay(1);
			if(counter_rx == MAX_TRY_REQ)
			{
				counter_rx = 0;
				return 0;
			}
		}
	}
}

uint8_t Send_Finish_OTA(uint32_t unicast_addr ,  uint8_t* buffer_packet, uint8_t* buffer_resp ,uint8_t ACK_REQ){
	uint32_t counter=0;
	buffer_packet[0] = BROADCAST_ADDR ; //unicast addr
	buffer_packet[1] = BROADCAST_ADDR ; // the  addr
	buffer_packet[2] = GW_SEND_DONE;
	buffer_packet[3] = ACK_REQ;
	AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
	ret = SX1278_LoRaEntryTx(&SX1278_1, SIZE_BUFFER_128BYTES, MAX_TIME_OUT);
	AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_packet, 128);
	while(1){
		ret = SX1278_LoRaTxPacket(&SX1278_1, (uint8_t*) buffer_packet, SIZE_BUFFER_128BYTES, MAX_TIME_OUT);
		if(ret){
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			ret = SX1278_LoRaEntryRx(&SX1278_1, SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
			return LORA_IF_GetData_End_Frame(&SX1278_1, (uint8_t*) buffer_resp, unicast_addr,SIZE_BUFFER_16BYTES, MAX_TIME_OUT);
		}
		else{
			counter++;
			if(counter == MAX_TRY_REQ)
			{
				return -1;
			}
		}

		}
}
// Using for testing flashing
uint32_t FL_u32ReadAddressData(uint32_t address){
	uint32_t Local_u32AddressData = *((volatile uint32_t*)(address));
	return Local_u32AddressData;
}

uint32_t FL_CalculateCRCFWLoRa(uint32_t u32Size_of_Image , uint32_t Active_Addr_Indicate){
	uint32_t local_u32CRC = 0;
	// Enable Clock for CRC
	RCC->AHBENR |=0x40;
	u32Size_of_Image = u32Size_of_Image/4;
	/* Resets the CRC calculation unit and set the data register to 0xFFFF_FFFF*/
	CRC->CR = 0x01;
	for(uint32_t Local_u32Count = 0U ; Local_u32Count < u32Size_of_Image ; Local_u32Count++){
		/* Calculate CRC */
		CRC->DR  = FL_u32ReadAddressData(Active_Addr_Indicate);
		Active_Addr_Indicate+=4;
	}
	local_u32CRC = CRC->DR;
	return local_u32CRC;
}
uint32_t FL_CalculateCRCBItmaskLoRa(uint32_t u32Size_of_bitmask , uint8_t *buffer){
	uint32_t Local_u32PlayloadCheck;
	// Enable Clock for CRC
	RCC->AHBENR |=0x40;
	u32Size_of_bitmask = u32Size_of_bitmask/4;
	uint32_t local_u32CRC ;
	/* Resets the CRC calculation unit and set the data register to 0xFFFF_FFFF*/
	CRC->CR = 0x01;
	for(uint32_t Local_u32Count = 0U ; Local_u32Count < u32Size_of_bitmask ; Local_u32Count++){
		Local_u32PlayloadCheck = (buffer[Local_u32Count*4+3] <<SHIFT_24_BIT)|(buffer[Local_u32Count*4+2] <<SHIFT_16_BIT)
										|(buffer[Local_u32Count*4+1] <<SHIFT_8_BIT)|(buffer[Local_u32Count*4] <<SHIFT_0_BIT);
		/* Calculate CRC */
		CRC->DR  = Local_u32PlayloadCheck;
	}
	local_u32CRC = CRC->DR;
	return local_u32CRC;
}
//
//void FL_u128PasteBuffer(uint8_t* buffer_packet , uint8_t no_packet)
//{
//	uint32_t value = 0U;
//	uint8_t i;
//	uint32_t FL_uint32localAddress_first =  (no_packet-1)*OFFSET_FLASHING_128BIT+ IMAGE_NEW_FIRMWARE;
//	// set all value off buffer_packet to 0xFF
//	memset((uint8_t*)buffer_packet , 0xff , 128U );
//	for(i =0 ; i < 32 ;i++){
//		value = FL_u32ReadAddressData(FL_uint32localAddress_first);
//		buffer_packet[i*4] = (value >> SHIFT_0_BIT) & 0xFF;
//		buffer_packet[1+4*i] = (value >> SHIFT_8_BIT) & 0xFF;
//		buffer_packet[2+4*i] = (value >> SHIFT_16_BIT) & 0xFF;
//		buffer_packet[3+4*i] = (value >> SHIFT_24_BIT) & 0xFF;
//		FL_uint32localAddress_first+=4;
//	}
//}

/*
 * @Request Start OTA
 */
void Send_Start_OTA(void){
	gl_u32TargetAddr = INITIAL_VALUE_ZERO;
	RTE_RUNNABLE_NODE_ADDR_ReadData(&gl_u32TargetAddr);
	if(Flashing_Request(ADDR_BROADCAST, gl_u32TargetAddr, buffer_req, GW_START_OTA , MCU_ACCEPT_RESPONSE) == 1){
		RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_WAIT_ACCEPT_OTA);
	}
}

/*
 * @Flashing Start
 */
void Wait_Start_OTA(void){
	uint16_t counter_tx =0;
	RTE_RUNNABLE_NODE_ADDR_ReadData(&gl_u32TargetAddr);
		RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_CONFIG_LORA);
	if(Wait_Accept_OTA(gl_u32TargetAddr,  (uint8_t*) buffer_resp , MCU_ACCEPT_RESPONSE ) == 1){

	}
	else {
		RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_REQUEST_OTA);
		RTE_RUNNABLE_PACKET_SEND_LORA_NUM_ReadData(&counter_tx);
		if(counter_tx == 10){
			RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_IDLE);
			RTE_RUNNABLE_UI_ERROR_WriteData(GW_OUTOFREQUEST_ERROR);
		}


	}


}


void FL_FlashLoRa_INIT(void){
 	gl_u8StatusFlash 			= 		INITIAL_VALUE_ZERO;
	gl_u16No_Packet_Flash		=		INITZERO_START+1;
	gl_start_array_backup 		= 		INITIAL_VALUE_ZERO;
	gl_u32CRCFW 				= 		INITIAL_VALUE_ZERO;
	gl_u16AppVer 	   			= 		INITIAL_VALUE_ZERO;
	gl_u32CodeSize 				=		INITIAL_VALUE_ZERO;
	gl_u8backup_flag 			=  		INITIAL_VALUE_ZERO;
	buffer_flashing_data        = 		NULL_PTR;
	//Calculate Number packet Lora SPI
	//Get Variable Via RTE FW
	//RTE_RUNNABLE_CRC_VALUE_ReadData(&gl_u32CRCFW);
	RTE_RUNNABLE_APP_VER_ReadData(&gl_u16AppVer);
	RTE_RUNNABLE_CODE_SIZE_ReadData(&gl_u32CodeSize);

	gl_u16NumberPacket_LoRa = (uint16_t)(gl_u32CodeSize/PACKET_64bytes)+1;
	RTE_RUNNABLE_NODE_ADDR_ReadData(&gl_u32TargetAddr);

	//Calculate CRC
	gl_u32CRCFW = FL_CalculateCRCFWLoRa(gl_u32CodeSize, IMAGE_NEW_FIRMWARE);
	RTE_RUNNABLE_CRC_VALUE_WriteData(gl_u32CRCFW);

	// Get the Infor SW FLASH
	GW_Config_SetUp();
	//Start Flashing
	// Pack inform to buffer
	// CRC
	buffer_req[0] = ADDR_BROADCAST;
	buffer_req[1] = ADDR_BROADCAST;
	buffer_req[15] = (gl_u32CRCFW>>SHIFT_24_BIT);
	buffer_req[14] = (gl_u32CRCFW>>SHIFT_16_BIT);
	buffer_req[13] = (gl_u32CRCFW>>SHIFT_8_BIT);
	buffer_req[12] = (gl_u32CRCFW>>SHIFT_0_BIT);
	// Appl ver
	buffer_req[11] =(gl_u16AppVer>>SHIFT_8_BIT);
	buffer_req[10] =(gl_u16AppVer>>SHIFT_0_BIT);
	// size Code
	buffer_req[9] =(gl_u32CodeSize>>SHIFT_24_BIT);
	buffer_req[8] =(gl_u32CodeSize>>SHIFT_16_BIT);
	buffer_req[7] =(gl_u32CodeSize>>SHIFT_8_BIT);
	buffer_req[6] =(gl_u32CodeSize>>SHIFT_0_BIT);
	// Node Addr
	if(FL_Syns_Config(gl_u32TargetAddr, (uint8_t*) buffer_req, (uint8_t*) buffer_resp, u8SF, u8BW, u8CR ) == FL_SUCCESS){

		/************Change SYSTEM TO SEND Update********************************/
		RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_ENCRYPT_FW);
		RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData(gl_u16No_Packet_Flash);
		HAL_Delay(1000);
		//GW_State_Save_State(SYS_ENCRYPT_FW);
	}
	//Flashing_end(ADDR_SLAVE_NODE_1,(uint8_t*)  buffer_req,(uint8_t*)  buffer_resp, number_packet);
}
uint16_t FL_getPacketlost(uint8_t* buffer_Rx , uint16_t packetNumber){
	uint16_t index =  (packetNumber)/8;
	uint16_t offset = (packetNumber)%8;
	return (buffer_Rx[index] >> offset) &0x01;
}
void FL_PacketLoRaSend_START(void){
	uint8_t Local_u8StatusFlash = INITIAL_VALUE_ZERO;

	// clear buffer
	clearDataBuffer((uint8_t*) buffer_req, 16);
	//getSizePacket(number_packet, (uint8_t*) buffer_req);
	if(gl_u16NumberPacket_LoRa>=0){
		//Get Data from EnCrypt cybertext
		RTE_RUNNABLE_ENCRYPT_DATA_BUFFER_ReadData(&buffer_flashing_data);
		//FL_u128PasteBuffer((uint8_t *)buffer_flashing_data ,gl_u8No_Packet_Flash);
		// Check is missing packet

		if (gl_u16NumberPacket_LoRa >0){
			//Send Packet Flashing
			Sequence_Process((uint8_t*)buffer_flashing_data );
			/*Display UI*/
			if(gl_u16NumberPacket_LoRa!=0)
				RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_ENCRYPT_FW);
			gl_u16NumberPacket_LoRa--;
			gl_u16No_Packet_Flash++;
		}
		if(gl_u8backup_flag == 1){
			// Get the lost message number
			gl_u16No_Packet_Flash = buffer_number_pack_lost[lost_counter_pos];
			lost_counter_pos++;
		}
		if(gl_u16No_Packet_Flash == 0xffff){
			gl_u8backup_flag = 0;
			gl_u16No_Packet_Flash = 0;
		}
		gl_u8start_array_backup++;
		RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData(gl_u16No_Packet_Flash);
		if(gl_u16NumberPacket_LoRa == 0){
			Local_u8StatusFlash = Flashing_end(ADDR_BROADCAST , gl_u32TargetAddr
								,(uint8_t*) buffer_packet , (uint8_t*) buffer_packet_Rx ,GW_ACKNOWLEDGE_FINISHING_SENDING_CODE );

			// Write status Flash to LORA RESP
			RTE_RUNNABLE_FLAG_LORA_RESP_WriteData(Local_u8StatusFlash);
			RTE_RUNNABLE_UI_ERROR_WriteData(Local_u8StatusFlash);
			switch (Local_u8StatusFlash) {
				case MCU_ACKNOWLEDGE_FINISHING:
					/*Flashing DONE */
					RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_DONE_UPDATE);
					break;
				case MCU_REQUEST_PACKET_FW_LOSS:
					//get the number packet loss

					memset(buffer_number_pack_lost , 0xffff , 64);
					uint16_t localcounter = INITIAL_VALUE_ZERO ;
					uint32_t CRC_bitmask_req =(buffer_packet_Rx[10] << SHIFT_24_BIT)|(buffer_packet_Rx[9] << SHIFT_16_BIT)
												|(buffer_packet_Rx[8] << SHIFT_8_BIT)|(buffer_packet_Rx[7] << SHIFT_0_BIT);
					copy_Array_BL(u8buffer_Pack_Lost, buffer_packet_Rx,11,88);
					if (FL_CalculateCRCBItmaskLoRa(88,u8buffer_Pack_Lost ) == CRC_bitmask_req){
						lost_counter_pos = 0;
						RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData(gl_u16No_Packet_Flash);
						for(uint16_t i=0 ; i < gl_u16No_Packet_Flash-1 ;i++){
							if(!FL_getPacketlost((uint8_t*) u8buffer_Pack_Lost , i )){
								buffer_number_pack_lost[localcounter] = i+1;
								localcounter++;
							}
						}
						//Get the number packet
						gl_u16NumberPacket_LoRa = localcounter;
						gl_u8backup_flag =1;
						lost_counter_pos++;
						// Init first packet
						gl_u16No_Packet_Flash = buffer_number_pack_lost[INITIAL_VALUE_ZERO];
						if(gl_u16No_Packet_Flash != 0xffff){
							RTE_RUNNABLE_PACKET_SEND_LORA_NUM_WriteData(gl_u16No_Packet_Flash);
							RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_ENCRYPT_FW);
						}
					}
					break;
				case MCU_ERROR_CRC :
					/* SEND UART NRC to ESP hear*/
					RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_IDLE);
					//HAL_UART_Transmit(&huart2, gl_u8ErrorFlag, 1, HAL_MAX_DELAY);
					break;
				default:
					/*Unknown Error*/
					break;
			}
		}
	}
}
void FL_PacketLoRaDone_OTA(void){
	GW_State_Save_State((uint32_t)SYS_IDLE);
	if(Flashing_end(ADDR_BROADCAST , gl_u32TargetAddr
									,(uint8_t*) buffer_packet , (uint8_t*) buffer_packet_Rx ,GW_ACKNOWLEDGE_END_OTA ) != MCU_ACKNOWLEDGE_FINISHING){
		RTE_RUNNABLE_PACKET_LORA_REIVECED_WriteData(buffer_packet_Rx);
		RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_IDLE);
	}
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	uint8_t flag_LoRa = 0;
//	Rte_FlagLoRaResp_ReadData(&flag_LoRa);
//	if(GPIO_Pin == GPIO_PIN_10 && flag_LoRa == 0x00){
//		uint32_t local_u32addrNode_req = 0;
//		ret = SX1278_LoRaRxPacket(&SX1278_2);
//		if ( ret > 0 ) {
//			ret = SX1278_read(&SX1278_2, (uint8_t*) buffer_resp_2, ret);
//			AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
//			AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_resp_2, 16);
//			//convert buffer to address_node_req
//			local_u32addrNode_req = (buffer_resp_2[0] << SHIFT_24_BIT) |(buffer_resp_2[1] << SHIFT_16_BIT)
//													 |(buffer_resp_2[2] << SHIFT_8_BIT) | (buffer_resp_2[3] << SHIFT_0_BIT);
//			if(local_u32addrNode_req == ADDRESS__MAC_NODE_1 ||local_u32addrNode_req == ADDRESS__MAC_NODE_2 ||
//							local_u32addrNode_req == ADDRESS__MAC_NODE_3 && buffer_resp_2[4] != GW_REQ_PARAMETER && buffer_resp_2[5] != GW_START_OTA )
//			{
//				HAL_UART_Transmit(&huart2, &buffer_resp_2, 16, HAL_MAX_DELAY);
//				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
//			}
//			memset(buffer_resp_2 , 0xff ,16);
//			}
//	}
//	else if(GPIO_Pin == GPIO_PIN_1 && flag_LoRa == 0x00 ){
//		uint32_t local_u32addrNode_req = 0;
//		ret = SX1278_LoRaRxPacket(&SX1278_1);
//		if ( ret > 0 ) {
//			ret = SX1278_read(&SX1278_1, (uint8_t*) buffer_resp_2, ret);
//			AES_init_ctx_iv(&ctx_req, AES_CTR_128_Key, AES_CTR_128_IV);
//			AES_CTR_xcrypt_buffer(&ctx_req, (uint8_t*) buffer_resp_2, 16);
//			//convert buffer to address_node_req
//			local_u32addrNode_req = (buffer_resp_2[0] << SHIFT_24_BIT) |(buffer_resp_2[1] << SHIFT_16_BIT)
//															 |(buffer_resp_2[2] << SHIFT_8_BIT) | (buffer_resp_2[3] << SHIFT_0_BIT);
//			if(local_u32addrNode_req == ADDRESS__MAC_NODE_1 ||local_u32addrNode_req == ADDRESS__MAC_NODE_2 ||
//					local_u32addrNode_req == ADDRESS__MAC_NODE_3)
//			{
//				HAL_UART_Transmit(&huart2, &buffer_resp_2, 16, HAL_MAX_DELAY);
//				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
//			}
//			memset(buffer_resp_2 , 0xff ,16);
//		}
//	}
//}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//  if (htim->Instance == TIM2) {
//	  RTE_RUNNABLE_FLAG_LORA_REQUEST_DEVICE_ReadData(&gl_FlagLoraSendDevice);
//	  if(gl_FlagLoraSendDevice == 0x00)
//		  RTE_RUNNABLE_FLAG_LORA_REQUEST_DEVICE_WriteData(0x01);
//	  else if(gl_FlagLoraSendDevice == 0x01)
//		  RTE_RUNNABLE_FLAG_LORA_REQUEST_DEVICE_WriteData(0x02);
//	  else if(gl_FlagLoraSendDevice == 0x02)
//		  RTE_RUNNABLE_FLAG_LORA_REQUEST_DEVICE_WriteData(0x03);
//	  else if(gl_FlagLoraSendDevice == 0x03)
//		  RTE_RUNNABLE_FLAG_LORA_REQUEST_DEVICE_WriteData(0x01);
  //}
}

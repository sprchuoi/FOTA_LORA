/*
 * Flashing_LoRa.h
 *
 *  Created on: Mar 26, 2024
 *      Author: quang
 */

#ifndef INC_FLASHING_LORA_H_
#define INC_FLASHING_LORA_H_
#include "SX1278_if.h"
#include "main.h"
#include "Flash_If.h"
#define ACTIVE_IMAGE              	(0x08005000)
#define OFFSET_FLASHING_128BIT		0x80
#define INITZERO_START				0U
typedef enum{
	FL_SUCCESS = 1,
	FL_FAIL =0 ,
	FL_ERROR = 2
} FL_Return_t;
FL_Return_t FL_Syns_Config(uint8_t address_update, uint8_t* buffer_packet, uint8_t* buffer_resp
		, uint8_t SF ,  uint8_t BandWidth , uint8_t CR );
FL_Return_t Flashing_init(uint8_t address_update, uint8_t* buffer_packet, uint8_t* buffer_resp , uint16_t number_packet);
//FL_Return_t Flashing_size(uint8_t address_update, uint8_t* buffer_packet, uint8_t* buffer_resp , uint32_t number_packet);
FL_Return_t Sequence_Process(uint8_t* buffer_flashing_data );
uint8_t Flashing_end(uint8_t unicast_addr,uint8_t addr_node, uint8_t* buffer_packet, uint8_t* buffer_resp , uint8_t ACK_REQ);
//void FL_u128PasteBuffer(uint8_t* buffer_packet , uint8_t no_packet );
uint32_t FL_u32ReadAddressData(uint32_t address);
void FL_FlashLoRa_INIT(void);
uint32_t FL_CalculateCRCFWLoRa(uint32_t u32Size_of_Image , uint32_t Active_Addr_Indicate);
void FL_PacketLoRaSend_START(void);
void FL_PacketLoRaDone_OTA(void);
void Send_Start_OTA(void);
uint8_t Send_Finish_OTA(uint8_t broadcast_addr ,uint8_t addr_node,
		uint8_t* buffer_packet, uint8_t* buffer_resp ,uint8_t ACK_REQ);
uint8_t Flashing_Request(uint8_t broadcast_addr ,uint8_t addr_node,
		uint8_t* buffer_packet ,uint8_t ACK_REQ , uint8_t ACK_RESP);
uint16_t FL_getPacketlost(uint8_t* buffer_Rx , uint16_t packetNumber);
uint8_t Wait_Accept_OTA(uint8_t broadcast_addr ,uint8_t addr_node,  uint8_t* buffer_resp ,uint8_t ACK_RESP );
void Wait_Start_OTA(void);
#endif /* INC_FLASHING_LORA_H_ */

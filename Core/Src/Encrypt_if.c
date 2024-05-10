/*
 * Encrypt_if.c
 *
 *  Created on: Apr 28, 2024
 *      Author: quang
 */
#include "Encrypt_if.h"
#include "Flash_If.h"


static uint8_t gl_u8CipherText[80];
static uint8_t gl_u8FwFragment[64];
static uint8_t gl_u8SystemState;
volatile uint32_t gl_ReadAddress;
static uint16_t gl_u16NoPacket;
static uint8_t gl_u8Flag_Packet;
static uint8_t gl_u8Addr_Packet;
static uint8_t gl_u8FlagActiveAddr;
static uint32_t gl_u32Version;
struct AES_ctx ctx_fw;
static uint8_t AES_CBC_128_Key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
static uint8_t AES_CBC_128_IV[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
void Encrypt_Address_Read_Init(){
	gl_ReadAddress = STORE_AREA_START_ADDRESS;
}

void Encrypt_MainFunc()
{
	Std_ReturnType retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_u8SystemState);
	retVal = RTE_RUNNABLE_PACKET_SEND_LORA_NUM_ReadData(&gl_u16NoPacket);
	gl_ReadAddress = STORE_AREA_START_ADDRESS +(gl_u16NoPacket-1)*64;
	//Initialize Cyper IV ctx
	AES_init_ctx_iv(&ctx_fw, AES_CBC_128_Key, AES_CBC_128_IV);
	if(RTE_E_OKE == retVal){
		if(SYS_ENCRYPT_FW == gl_u8SystemState){
			for(uint8_t local_counter  = 0 ; local_counter < 64 ; local_counter++ ){
				gl_u8FwFragment[local_counter] = (*(volatile uint8_t*)(gl_ReadAddress));
				gl_ReadAddress++;
			}
			RTE_RUNNABLE_PACKET_SEND_LORA_NUM_ReadData(&gl_u16NoPacket);
			RTE_RUNNABLE_NODE_ADDR_ReadData(&gl_u8Addr_Packet);
			gl_u8CipherText[0]= ADDR_BROADCAST;
			gl_u8CipherText[1]= gl_u8Addr_Packet;
			gl_u8CipherText[2] =FL_FRAGMENT_FIRMWARE;
			gl_u8CipherText[3]= gl_u16NoPacket >> SHIFT_8_BIT;
			gl_u8CipherText[4]= gl_u16NoPacket >> SHIFT_0_BIT;
			copy_Array((uint8_t*)gl_u8CipherText ,(uint8_t*) gl_u8FwFragment , 64);

			/*Encrypt buffer to cyberText*/
			AES_CTR_xcrypt_buffer(&ctx_fw, gl_u8CipherText, 80);
			retVal = RTE_RUNNABLE_ENCRYPT_DATA_BUFFER_WriteData(gl_u8CipherText);
			if(RTE_E_OKE == retVal){
				// Set to Send Update
				RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_SEND_UPDATE);
			}
		}

	}
}


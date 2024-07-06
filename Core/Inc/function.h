/*
 * function.h
 *
 *  Created on: Apr 18, 2024
 *      Author: quang
 */
#ifndef _FUNCTION_H_
#define _FUNCTION_H_
#include "main.h"
#define NUMBER_OF_FLAGS 20U
#define PAGE_NUMBER_IN_FLAG_REGION    127U
#define WORD_SIZE_IN_BYTE                     4U
#define BL_FLASH
#define START_OF_FLAG_REGION                  0x0801FC00
#define END_OF_FLAG_REGION                    0x0801FC60
#define BL_BRANCHING_FLAG_SET                       0x00000000
#define BL_BRANCHING_FLAG_RESET                     0xFFFFFFFF
#define BL_FLASHING_STATE						(0x0801FC00)
#define FLAG_STATUS_BOOTLOADER                (START_OF_FLAG_REGION)
#define FLAG_IMAGE							   FLAG_STATUS_BOOTLOADER
#define ERASED_VALUE                          0xffffffff

#define BANKFIRST_IMAGE              	(uint32_t)(0x08005000)      // Origin + Bootloader size (20kB)
#define FIRST_IMAGE_START_ADDRESS	BANKFIRST_IMAGE
#define BANKSECOND_IMAGE              	(uint32_t)(0x0800A800)      // Origin + Bootloader size (20kB) + Active Bank (22kB)
#define SECOND_IMAGE_START_ADDRESS	BANKSECOND_IMAGE
#define FLAG_NODE_ID												(0x0801FC04)
#define FLAG_STATUS_BANKFIRST_APP_VER_ADDRESS                       (0x0801FC10)
#define FLAG_STATUS_BANKFIRST_REGION_ADDRESS                        (0x0801FC14)
#define FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS                   (0x0801FC18)
#define FLAG_STATUS_CRC_BANKFIRST_REGION_ADDRESS                    (0x0801FC1C)
#define FLAG_STATUS_ENTRY_POINT_VALUE_BANKFIRST_REGION_ADDRESS      (0x0801FC20)
//Status region Bank 2
#define FLAG_STATUS_BANKSECOND_APP_VER_ADDRESS                      (0x0801FC30)
#define FLAG_STATUS_BANKSECOND_REGION_ADDRESS                 		(0x0801FC34)
#define FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS                  (0x0801FC38)
#define FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS                   (0x0801FC3C)
#define FLAG_STATUS_ENTRY_POINT_VALUE_BANKSECOND_REGION_ADDRESS     (0x0801FC40)

#define FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS							(0x0801FC44)


void FUNC_Blink_Led_Receive(uint32_t TIMER);

void FUNC_Blink_Led_Send(uint32_t TIMER);

void FUNC_EraseAndRestore_Header_Page(uint32_t Copy_u32Address, uint32_t Copy_u32NewData);

void FUNC_get_DHT_val(uint8_t * buffer);

uint16_t FUNC_get_MQ_val();

void FUNC_voidMakeSoftWareReset(void);
uint32_t FUNC_ReaddataAddress(uint32_t Address);
#endif /*_FUNCTION_H_*/


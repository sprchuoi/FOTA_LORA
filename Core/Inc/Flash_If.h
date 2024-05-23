/*
 * Flash_Interface.h
 *
 *  Created on: Apr 28, 2024
 *      Author: quang
 */

#ifndef INC_FLASH_IF_H_
#define INC_FLASH_IF_H_

#define STORE_AREA_START_ADDRESS 0x08010000
#define IMAGE_NEW_FIRMWARE 	STORE_AREA_START_ADDRESS

#define INITIAL_ZERO 0x0

#define FLASH_ADDRESS_STEP	2

#include "main.h"
void F_voidInitVariables(void);
HAL_StatusTypeDef F_Erase_Image(uint32_t ImageAddress);
HAL_StatusTypeDef F_FlashHalfWordToAddress(uint32_t Copy_Address , uint16_t Copy_u16DataAddress);
HAL_StatusTypeDef F_FlashWordToAddress(uint32_t Copy_Address , uint32_t Copy_u32DataAddress);
HAL_StatusTypeDef F_FlashBlockToAddress(const uint8_t *pData , uint16_t SizeOfDataBuffer);


#endif /* INC_FLASH_IF_H_ */

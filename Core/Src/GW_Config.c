/*
 * GW_Config.c
 *
 *  Created on: Apr 7, 2024
 *      Author: quang
 */
#include "GW_Config.h"
#include "SX1278.h"
#include "main.h"


// Global Parameter
uint8_t u8BW; // Bandwidth
uint8_t u8SF; // Spreading Frequency
uint8_t u8CR; // Coding Rate
uint32_t GW_Config_GetParameter(uint32_t Address){
	return *((volatile uint32_t*)(Address));
}


void GW_voidEraseRestoreConfigPage(uint32_t Copy_u32Address, uint32_t Copy_u32NewData)
{
	uint32_t Local_u32AddressArray	[NUMBER_OF_FLAGS];
	uint32_t Local_u32DataArray		[NUMBER_OF_FLAGS];
	uint16_t Local_u16DataIndex        = 0;
	uint16_t Local_u16DataCounter      = 0;
	uint32_t Local_u32AddressCounter   = 0;

	//Copy all flag to array before erase
	for( Local_u32AddressCounter = GW_START_OF_FLAG_ADDR ;Local_u32AddressCounter < GW_END_OF_FLAG_ADDR;)
	{
		if( (Local_u32AddressCounter != Copy_u32Address) & (*((volatile uint32_t*)(Local_u32AddressCounter)) != ERASED_VALUE))
		{
			Local_u32AddressArray[Local_u16DataIndex] = Local_u32AddressCounter;
			Local_u32DataArray[Local_u16DataIndex] = *((volatile uint32_t*)(Local_u32AddressCounter));
			Local_u16DataIndex++ ;
		}
		Local_u32AddressCounter = Local_u32AddressCounter + WORD_SIZE_IN_BYTE;
	}

	// Erase the Flag region.
	FLASH_EraseInitTypeDef Local_eraseInfo;
	uint32_t Local_u32PageError;
	Local_eraseInfo.TypeErase = FLASH_TYPEERASE_PAGES;
	Local_eraseInfo.Banks = FLASH_BANK_1;
	Local_eraseInfo.PageAddress = FLAG_STATUS_GW_CONFIG;
	Local_eraseInfo.NbPages = 1;

	HAL_FLASH_Unlock(); //Unlocks the flash memory
	HAL_FLASHEx_Erase(&Local_eraseInfo, &Local_u32PageError); //Deletes given sectors
	for (Local_u16DataCounter = 0 ; Local_u16DataCounter < Local_u16DataIndex ; Local_u16DataCounter++ )
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Local_u32AddressArray[Local_u16DataCounter], Local_u32DataArray[Local_u16DataCounter]);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Copy_u32Address, Copy_u32NewData); //Replace new data to flash
	HAL_FLASH_Lock();  //Locks again the flash memory
}


void GW_Config_Init(void){
	SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, SX1278_LORA_SF_12,
				SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 16);
}
void GW_Config_SetUp(void){
	uint32_t GW_u32LocalStatus_GW_Config = GW_Config_GetParameter(FLAG_STATUS_GW_CONFIG);
	uint32_t GW_u32LocalParameter= GW_Config_GetParameter(FLAG_PARAMETER_GW_CONFIG);
	// Get Parameter

	if(GW_u32LocalStatus_GW_Config == GW_CONFIG_PARAMETER_RESET ){

		// SET CONFIG Value as Default
		GW_voidEraseRestoreConfigPage(FLAG_PARAMETER_GW_CONFIG ,GW_CONFIG_PARAMETER_SF_BW_CR_DEFAULT );
	}
	if(GW_u32LocalStatus_GW_Config == GW_CONFIG_PARAMETER_SET ){
		// do nothing
	}
	u8SF = (GW_u32LocalParameter >> SHIFT_16_BIT)& 0xFF ;
	u8BW = (GW_u32LocalParameter >> SHIFT_8_BIT)& 0xFF ;
	u8CR = (GW_u32LocalParameter >> SHIFT_0_BIT)& 0xFF ;
}



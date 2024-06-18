/*
 * BL_Program.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Sprchuoi
 */


/*******************************************************************************
 * Title                 :   BL
 * Filename              :   BL_Program.c
 * Author                :   Chau_Thanh_Dat && Nguyen_Quang_Binh
 * Origin Date           :   19/1/2024
 * Version               :   1.0.0
 * Compiler              :   GCC (STM32CubeIDE)
 * Target                :   STM32F103C8T6
 * Notes                 :   None
 *
 *****************************************************************************/

//#define Debug
//**************************Include***************************//
#include "BL_Program.h"
#include "SX1278_if.h"
#include <string.h>
uint8_t buffer_req[16];
uint8_t buffer_flashing_data[64];
uint8_t buffer_mark_packet_loss[112];
uint8_t buffer_lost_map[512];
uint8_t buffer_packet[80];
uint32_t Local_u32SizeOfCode;
uint8_t buffer_resp[16];
uint16_t  Local_u16index_fragment;
uint32_t u32Buffer_Flash[16];
uint32_t BL_CRCCheck;
uint8_t u8CHANNEL;
uint8_t u8SF;
uint8_t u8BW;
uint8_t u8CR;
uint16_t packet_current;
uint32_t gl_sizecode;
uint8_t ret;
uint16_t gl_totalPacket;
uint8_t gl_RSSI;
uint8_t gl_SNR;
uint8_t gl_State_BL;
static  uint8_t AES_CBC_128_Key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
static  uint8_t AES_CBC_128_IV[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
struct AES_ctx ctx_fw;
static Bitmask bm;
//**************************Include***************************//
uint32_t BL_u32ReadAddressData(uint32_t address){
	uint32_t Local_u32AddressData = *((volatile uint32_t*)(address));
	return Local_u32AddressData;
}
// Set default baudrate in bootloader
void BL_voidSetConfigLoRa(){
	SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, SX1278_LORA_SF_12,
			SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 16);
}
//**************************Function Define***************************//
uint8_t BL_Check_CRC(uint32_t CRC_expect , uint8_t *buffer_check){
	uint32_t Local_u32PlayloadCheck = BL_INITIALIZE_WITH_ZERO;
	RCC->AHBENR |=0x40;
	/* Resets the CRC calculation unit and set the data register to 0xFFFF_FFFF*/
	CRC->CR = 0x01;
	for(uint8_t Local_u8Count = 0U ; Local_u8Count < 16U ; Local_u8Count++){
		/* Calculate CRC */
		Local_u32PlayloadCheck = (buffer_check[Local_u8Count*4+3] <<SHIFT_24_BIT)|(buffer_check[Local_u8Count*4+2] <<SHIFT_16_BIT)
								|(buffer_check[Local_u8Count*4+1] <<SHIFT_8_BIT)|(buffer_check[Local_u8Count*4] <<SHIFT_0_BIT);
		CRC->DR  = Local_u32PlayloadCheck;
	}
	if((CRC->DR) == CRC_expect ){
				return BL_OK;
	}
	return BL_CHKS_ERROR;
}

uint32_t BL_Calculate_CRC(uint8_t *buffer , uint32_t lenght){
	uint32_t Local_u32PlayloadCheck = BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32CRC;
		RCC->AHBENR |=0x40;
		/* Resets the CRC calculation unit and set the data register to 0xFFFF_FFFF*/
		CRC->CR = 0x01;
		for(uint8_t Local_u8Count = 0U ; Local_u8Count < lenght/32 ; Local_u8Count++){
			/* Calculate CRC */
			Local_u32PlayloadCheck = (buffer[Local_u8Count*4+3] <<SHIFT_24_BIT)|(buffer[Local_u8Count*4+2] <<SHIFT_16_BIT)
									|(buffer[Local_u8Count*4+1] <<SHIFT_8_BIT)|(buffer[Local_u8Count*4] <<SHIFT_0_BIT);
			CRC->DR  = Local_u32PlayloadCheck;
		}
		Local_u32CRC = CRC->DR;
		return Local_u32CRC;
}
uint32_t BL_Read_Address_Node(){
	uint32_t Local_u32AddressData = *((volatile uint32_t*)(FLAG_INDICATE_ADDRESS_NODE));
	return Local_u32AddressData;
}
void BL_voidCopyImageToActiveRegion(void){
	FLASH_EraseInitTypeDef Local_eraseInfo;
	uint32_t Local_u32PageError;
	uint32_t Local_u32BackupDataAddress = BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32ActiveDataAddress = BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32BackUpDataWord 	= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32BackupSizeInWord 	= BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS);
	uint32_t Local_u32CRCActiveBank     = BL_u32ReadAddressData(FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS);
	//set active img to corrupt make sure install again
	BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_REGION_ADDRESS , BR_SET_IMAGE_CORRUPTED );
	uint32_t* Local_u32BackupImagePointer = (uint32_t*)DOWNLOAD_BANK_START_ADDRESS;
	Local_u32BackupSizeInWord = Local_u32BackupSizeInWord / 4;
	// Erase the Active region.
	Local_eraseInfo.TypeErase = FLASH_TYPEERASE_PAGES;
	Local_eraseInfo.Banks = FLASH_BANK_1;
	Local_eraseInfo.PageAddress = BANKFIRST_IMAGE;
	Local_eraseInfo.NbPages =	FLASH_BANK_NUMOFPAGE;

	HAL_FLASH_Unlock(); //Unlocks the flash memory
	HAL_FLASHEx_Erase(&Local_eraseInfo, &Local_u32PageError); //Deletes given sectors
	HAL_FLASH_Lock();  //Locks again the flash memory

	//Copy data from download to active region.
	HAL_FLASH_Unlock();
	for(uint32_t Local_uint32Count = 0 ; Local_uint32Count  < Local_u32BackupSizeInWord ; Local_uint32Count++)
	{
		Local_u32ActiveDataAddress = (BANKFIRST_IMAGE + (WORD_SIZE_IN_BYTE * Local_uint32Count));
		Local_u32BackupDataAddress = (DOWNLOAD_BANK_START_ADDRESS + (WORD_SIZE_IN_BYTE * Local_uint32Count));
		Local_u32BackUpDataWord    = *((volatile uint32_t*)(Local_u32BackupDataAddress));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Local_u32ActiveDataAddress, Local_u32BackUpDataWord);
	}
		HAL_FLASH_Lock();
		BL_voidEraseRestoreHeaderPage(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS , Local_u32BackupSizeInWord*4);
		BL_voidEraseRestoreHeaderPage(FLAG_STATUS_CRC_BANKFIRST_REGION_ADDRESS , Local_u32CRCActiveBank);
		BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_REGION_ADDRESS , BR_SET_IMAGE_ACTIVE );
}
void BL_voidBootLoader_Init(void)
{
	//BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER,BL_BRANCHING_FLAG_SET);
	// Read Branching Request Update Flag.
	// UI init boot
	uint32_t Local_u32Flag = BL_u32ReadAddressData(FLAG_STATUS_BOOTLOADER);
	if(Local_u32Flag == BL_BRANCHING_FLAG_RESET)
	{
		//Initialize MAC ADDRESS
		BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ADDRESS_NODE , SENSOR_ADDRESS_MAC);
		// Check images existence, status (and CRC).
		BL_voidCheckActiveRegion();
	}
	else if(Local_u32Flag == BL_BRANCHING_FLAG_SET)
	{
		// Goto Boot_loader to Receive code in the inactive image.
	    BL_voidJumpToBootloader();
	}
	else
	{
		// Application  not exited and backup image not exit
		BL_voidEraseRestoreHeaderPage(START_OF_FLAG_REGION , BL_BRANCHING_FLAG_SET);
		// Reset bootloader
		NVIC_SystemReset();
	}
}
uint8_t BL_CheckSize(uint32_t size_app){
	return( size_app <= APP_ACTIVE_SIZE ) ? BL_OK : BL_SIZE_ERROR;
}
/**
 * @brief  This function verifies the checksum of application located in flash.
 *         If ::USE_CHECKSUM configuration parameter is disabled then the
 *         function always returns an error code.
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: if calculated checksum matches the application checksum
 * @retval BL_CHKS_ERROR: upon checksum mismatch or when ::USE_CHECKSUM is
 *         disabled
 */
uint8_t BL_VerifyCheckSum(uint32_t u32Size_of_Image , uint32_t CRC_CODE, uint32_t Active_Addr_Indicate){
	#if(USE_CHECKSUM)
		// Enable Clock for CRC
		// set the mark for the active addr
		RCC->AHBENR |=0x40;
		u32Size_of_Image = u32Size_of_Image/4;
		/* Resets the CRC calculation unit and set the data register to 0xFFFF_FFFF*/
		CRC->CR = 0x01;
		for(uint32_t Local_u32Count = 0U ; Local_u32Count < u32Size_of_Image ; Local_u32Count++){
			/* Calculate CRC */
			CRC->DR  = BL_u32ReadAddressData(Active_Addr_Indicate);
			// Recall calculate for CRC IMG bank first
			Active_Addr_Indicate+=0x04u;
			}
	#endif
		// Check Valid Image
		if((CRC->DR) == CRC_CODE ){
			return BL_OK;
		}
		return BL_CHKS_ERROR;

}

HAL_StatusTypeDef BL_voidEraseBank(uint32_t Address_Flash)
{
	HAL_StatusTypeDef	Local_retVal;
	if (Address_Flash == BR_SET_IMAGE_NOT_EXISTING ){
		Address_Flash = BANKFIRST_IMAGE;
	}
	FLASH_EraseInitTypeDef Local_eraseInfo;
	uint32_t Local_u32PageError;
	Local_eraseInfo.TypeErase = FLASH_TYPEERASE_PAGES;
	Local_eraseInfo.Banks = FLASH_BANK_1;
	Local_eraseInfo.PageAddress = Address_Flash;
	Local_eraseInfo.NbPages =	44;

	HAL_FLASH_Unlock(); //Unlocks the flash memory
	Local_retVal = HAL_FLASHEx_Erase(&Local_eraseInfo, &Local_u32PageError); //Deletes given sectors

	HAL_FLASH_Lock();
	return Local_retVal;
}

void initBitMask(Bitmask *bm){
	for (uint32_t i = 0; i < (NUM_PACKETS_MAX+7) / 8; i++) {
		bm->bitmask[i] = 0;
	}
}

void setBit_BitMask(Bitmask *bm , uint16_t packetNumber){
	uint8_t index = (packetNumber-1) / 8;
	uint8_t offset = (packetNumber-1) % 8;
	bm->bitmask[index] |= (1 << offset);
}
void BL_voidCheckImgCorrectBankFirst(void){
	uint32_t Local_u32ActiveImageStatus = BL_u32ReadAddressData(FLAG_STATUS_BANKFIRST_REGION_ADDRESS);
	uint32_t Local_u32ReceivedCRC       = BL_u32ReadAddressData(FLAG_STATUS_CRC_BANKFIRST_REGION_ADDRESS);
	uint32_t Local_u32BackupStatus      = BL_INITIALIZE_EITH_CORRUPTED;
	uint32_t Local_u32SizeOfImageActive = BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS);
	uint32_t Local_u32ActiveRegion      = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	if(Local_u32ActiveImageStatus == BR_IMAGE_IS_ACTIVE  )
		{
			//Verify CheckSum
	    	if(BL_VerifyCheckSum(Local_u32SizeOfImageActive, Local_u32ReceivedCRC ,FIRST_IMAGE_START_ADDRESS ) == BL_OK){
	    		BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS, BANKFIRST_IMAGE);
	    		BL_voidJumpToActiveRegion();
	    	}
	    	else {
	    		BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_REGION_ADDRESS , BR_SET_IMAGE_CORRUPTED);
	    		// Check Image bank 2
	    		//BL_voidMakeSoftWareReset();
	    	}

		}
	else if(Local_u32ActiveImageStatus == BR_IMAGE_IS_CORRUPTED || Local_u32ActiveImageStatus == BR_IMAGE_IS_NOT_EXISTING)
		{
	    	Local_u32BackupStatus    = BL_32CheckBankSecondRegion();
			// Check the status of the backup image if the image in he active region is corrupted or not exisit.
			if(Local_u32BackupStatus == BR_IMAGE_IS_CORRECT)
			{
				// update braching flag to bank first
				BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS , BANKSECOND_IMAGE);
				//Reset SW
				BL_voidJumpToActiveRegion();
			}
			else if(Local_u32BackupStatus == BR_IMAGE_IS_CORRUPTED || Local_u32BackupStatus == BR_IMAGE_IS_NOT_EXISTING)
			{
				// Set branching flag to go to boot loader to receive a new code as all codes in the flash corrupted.
				BL_voidSetBranchingFlagAndMakeSWR();
			}
		}
	    else
	    {
	        // Do nothing
	    }
}
void BL_voidCheckImgCorrectBankSecond(void){
	uint32_t Local_u32ActiveImageStatus = BL_u32ReadAddressData(FLAG_STATUS_BANKSECOND_REGION_ADDRESS);
	uint32_t Local_u32ReceivedCRC       = BL_u32ReadAddressData(FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS);
	uint32_t Local_u32BackupStatus      = BL_INITIALIZE_EITH_CORRUPTED;
	uint32_t Local_u32SizeOfImageActive = BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS);
	uint32_t Local_u32ActiveRegion      = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	if(Local_u32ActiveImageStatus == BR_IMAGE_IS_ACTIVE)
	{
		//Verify CheckSum
		if(BL_VerifyCheckSum(Local_u32SizeOfImageActive, Local_u32ReceivedCRC , DOWNLOAD_BANK_START_ADDRESS) == BL_OK ){
			BL_voidCopyImageToActiveRegion();
			BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS, BANKFIRST_IMAGE);
			BL_voidMakeSoftWareReset();
		}
		else {
			BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKSECOND_REGION_ADDRESS , BR_SET_IMAGE_CORRUPTED);
			// set to jump to boot
			BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER , BL_BRANCHING_FLAG_SET);
			BL_voidMakeSoftWareReset();
		}

	}
	else if(Local_u32ActiveImageStatus == BR_IMAGE_IS_CORRUPTED || Local_u32ActiveImageStatus == BR_IMAGE_IS_NOT_EXISTING)
	{

		// Set branching flag to go to boot loader to receive a new code as all codes in the flash corrupted.
		BL_voidSetBranchingFlagAndMakeSWR();
	}
	else
	{
		// Error Code

	}
}

void BL_voidCheckActiveRegion(void)
{
    // Read Images Status To Determine Which Image Will Be Excuted.
	uint32_t Local_u32ActiveRegionRunning = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	switch(Local_u32ActiveRegionRunning)
	{
		case BANKFIRST_IMAGE:
			BL_voidCheckImgCorrectBankFirst();
			break;
		case BR_SET_IMAGE_NOT_EXISTING:
			BL_voidCheckImgCorrectBankSecond();
		default :
			/*ERROR*/
			break;

	}
	// Reset SW
	BL_voidSetBranchingFlagAndMakeSWR();

}

uint32_t BL_32CheckBankFirstRegion(void)
{
	uint32_t Local_u32BackupStatus =BL_u32ReadAddressData(FLAG_STATUS_BANKFIRST_REGION_ADDRESS);
	// if exist Backup image
	if(Local_u32BackupStatus == BR_IMAGE_IS_ACTIVE)
	{
		return BR_IMAGE_IS_CORRECT;
	}
	else if(Local_u32BackupStatus == BR_IMAGE_IS_CORRUPTED || Local_u32BackupStatus == BR_IMAGE_IS_NOT_EXISTING)
	{
		return BR_IMAGE_IS_CORRUPTED;
	}
	else
	{
		//Do nothing here
	}
	return Local_u32BackupStatus;
}

uint32_t BL_32CheckBankSecondRegion(void)
{
	uint32_t Local_u32BackupStatus =BL_u32ReadAddressData(FLAG_STATUS_BANKSECOND_REGION_ADDRESS);
	// if exist Backup image
	if(Local_u32BackupStatus == BR_IMAGE_IS_ACTIVE)
	{
		return BR_IMAGE_IS_CORRECT;
	}
	else if(Local_u32BackupStatus == BR_IMAGE_IS_CORRUPTED || Local_u32BackupStatus == BR_IMAGE_IS_NOT_EXISTING)
	{
		return BR_IMAGE_IS_CORRUPTED;
	}
	else
	{
		//Do nothing here
	}
	return Local_u32BackupStatus;
}


void BL_voidJumpToActiveRegion(void)
{
	uint32_t Local_u32ActiveImageAddress = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	//MX_GPIO_Deinit();
	SysTick->CTRL = 0x0; //Disables SysTick timer and its related interrupt
	HAL_DeInit();
	RCC->CIR = 0x00000000; //Disable all interrupts related to clock
	//uint32_t Local_u32ActiveImageAddress  = *(Application_t*)(Local_u32ActiveImageAddress + 4) ;
	Application_t AddressToCall = 0 ;
	AddressToCall = *(Application_t*)(Local_u32ActiveImageAddress + 4); // Point to Reset Handler
	//AddressToCall = Local_u32ActiveImageAddress;
	__DMB(); //ARM says to use a DMB instruction before relocating VTOR *
	SCB->VTOR = BANKFIRST_IMAGE; //We relocate vector table to the sector 1 of Active Region
	__DSB(); //ARM says to use a DSB instruction just after 	relocating VTOR */

	AddressToCall();
}

void BL_voidJumpToBootloader(void)
{
	//@TODO: In develop
	BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER , BL_BRANCHING_FLAG_RESET);
	BL_voidUpdateHeaders();
	//BL_voidReceiveUpdate();
}

void BL_voidEraseRestoreHeaderPage(uint32_t Copy_u32Address, uint32_t Copy_u32NewData)
{
	uint32_t Local_u32AddressArray	[NUMBER_OF_FLAGS];
	uint32_t Local_u32DataArray		[NUMBER_OF_FLAGS];
	uint16_t Local_u16DataIndex        = 0;
	uint16_t Local_u16DataCounter      = 0;
	uint32_t Local_u32AddressCounter   = 0;

	//Copy all flag to array before erase
	for( Local_u32AddressCounter = START_OF_FLAG_REGION ;Local_u32AddressCounter < END_OF_FLAG_REGION;)
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
	Local_eraseInfo.PageAddress = FLAG_IMAGE;
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

void BL_voidSetBranchingFlagAndMakeSWR(void)
{
	// Set Branching Flag To Receive New Code.
	BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER, BL_SET_BRANCHING_FLAG);
	// Make Software Reset.
	//BL_voidMakeSoftWareReset();
	NVIC_SystemReset();
}


/*Update size of Image in header*/
void BL_voidUpdateHeaders(void)
{
	uint32_t Local_u32ImageSizeInBytes         = BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32CRCImage 					=BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32VerImage 					= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_counter 						= BL_INITIALIZE_WITH_ZERO;
	//Structure LoRa Transmit
	BL_voidSetConfigLoRa();
	// get Config Synchronize
	/* MCU response MCU_ENTER_FBL to GW and expect get resp as GW_SYNC_CONFIG*/



	while(1){
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		LORA_IF_Stransmit_Response(&SX1278, buffer_resp, ret, ADDR_NODE_1, MCU_ACCEPT_REQUEST);
		//Change State In hear
		gl_State_BL = STATE_INITBOOT;
		__HAL_GPIO_EXTI_CLEAR_IT(DIO0_Pin);
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		ret = SX1278_LoRaEntryRx(&SX1278, SIZE_BUFFER_16BYTES , 10000);
		HAL_Delay(2000);
		// Wait to get GW CONFIG
		// Request LoRa stransmit to get size of code
		if(gl_State_BL == STATE_RECEIVE_HEADER)
		{
			//Send response to GW
			LORA_IF_TransferData_Frame(&SX1278, (uint8_t*)  buffer_req, ret, MAX_TIME_OUT, SIZE_BUFFER_16BYTES, MCU_RECEIVED_CONFIG);
			HAL_NVIC_DisableIRQ(EXTI1_IRQn);
			//Set Parameter and Configurate for LoRa
			u8SF= buffer_resp[3];
			u8BW= buffer_resp[4];
			u8CR= buffer_resp[5];
			/*Response Goto Programming and Send Update Request */
			SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, u8SF,
									u8BW, u8CR, SX1278_LORA_CRC_EN, 128);
			//HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			// change to receive buffer 80 bytes
			//ret = SX1278_LoRaEntryRx(&SX1278, SIZE_BUFFER_80BYTES , 10000);
			/*Get the size code */
			Local_u32ImageSizeInBytes = (buffer_resp[9] << SHIFT_24_BIT) | (buffer_resp[8] << SHIFT_16_BIT) |
					(buffer_resp[7] << SHIFT_8_BIT) | (buffer_resp[6] << SHIFT_0_BIT);

			/*Get the Version Img*/
			Local_u32VerImage = (buffer_resp[11]<<SHIFT_8_BIT | buffer_resp[10] << SHIFT_0_BIT );
			/*Get the CRC */
			Local_u32CRCImage = (buffer_resp[15] << SHIFT_24_BIT) | (buffer_resp[14] << SHIFT_16_BIT) |
					(buffer_resp[13] << SHIFT_8_BIT) | (buffer_resp[12] << SHIFT_0_BIT);
			// Get total packets
			//gl_totalPacket  = (buffer_resp[11]<<SHIFT_8_BIT) | (buffer_resp[10] <<SHIFT_0_BIT);
			/* MCU response MCU_ENTER_FBL to GW and expect get resp as GW_SYNC_CONFIG*/

			if(BL_CheckSize(Local_u32ImageSizeInBytes) == BL_CHKS_ERROR){
				LORA_IF_TransferData_Frame(&SX1278 , (uint8_t*) buffer_resp , ret , MAX_TIME_OUT ,SIZE_BUFFER_16BYTES , BL_LARGE_SIZE );
				BL_voidSetBranchingFlagAndMakeSWR();
			}
			else{
				// Repare for downloading to bank download
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKSECOND_APP_VER_ADDRESS,Local_u32VerImage );
				//Set CRC
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS,Local_u32CRCImage);
				//set Bank Second IMG Corrupted - if flashing success update img correct
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKSECOND_REGION_ADDRESS,BR_SET_IMAGE_CORRUPTED);
				//update imgsize
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS,Local_u32ImageSizeInBytes);
				BL_voidEraseBank(BANKSECOND_IMAGE);
				// update for bank second
				}
				BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS , BANKFIRST_IMAGE);
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER , BL_RESET_BRANCHING_FLAG);
				BL_voidReceiveUpdate();
			}
	}
}

void BL_voidReceiveUpdate(void)
{
	uint32_t Local_u32ExpectActiveAddr 							= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32OffsetVector_H							= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32OffsetVector_L							= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32HighByteDataReceive  						= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32LowByteDataReceive  						= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32PageError;
	uint32_t Local_u32InactiveImageAddressCounter_Base			= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32InactiveImageAddressCounter 				= BL_INITIALIZE_WITH_ZERO;
	Local_u16index_fragment							  	        = BL_INITIALIZE_WITH_ZERO;
	uint16_t  Local_u16index_fragment_previous					= BL_INITIALIZE_WITH_ZERO;
	uint16_t  Local_u16counter_packet_loss						= BL_INITIALIZE_WITH_ZERO;
	uint32_t local_u32_CRC_bitmask								= BL_INITIALIZE_WITH_ZERO;
	FLASH_EraseInitTypeDef Local_eraseInfo;
	uint32_t Local_u32ActiveRegionRunning                       = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	uint32_t local_unicast_addr									= BL_Read_Address_Node();
	uint16_t Local_u16MaxPacket 								= BL_INITIALIZE_WITH_ZERO;
	uint16_t local_totalPacket									=BL_INITIALIZE_WITH_ZERO;
	uint32_t local_u32CRC_fragment_firmware						= BL_INITIALIZE_WITH_ZERO;
	//Get the info from bank 2
	Local_u32SizeOfCode 								= BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS);
	Local_u32InactiveImageAddressCounter_Base               = DOWNLOAD_BANK_START_ADDRESS;
	Local_u32ExpectActiveAddr 								= FIRST_IMAGE_START_ADDRESS;
	uint8_t Local_Flag 										=	BL_INITIALIZE_WITH_ZERO;
	//Loop to receive code update
	memset((uint8_t*) buffer_mark_packet_loss ,0xff , 96 );
	local_totalPacket = (uint16_t)((Local_u32SizeOfCode + 64 -1)  /64);


	Local_u16MaxPacket = local_totalPacket;
	// Init bitmask
	initBitMask(&bm);
	while(local_totalPacket>=0 && local_totalPacket != 0xFFFF )
	{
		/*Initial in the first time start up speed*/
		if(Local_u16index_fragment == 0)
			SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, u8SF,
												u8BW, u8CR, SX1278_LORA_CRC_EN, 128);
		AES_init_ctx_iv(&ctx_fw, AES_CBC_128_Key, AES_CBC_128_IV);
		memset((uint8_t*)buffer_packet , 0xff , 80);
		//Receive code update Fragment firmware
		Local_u16index_fragment = LORA_IF_GetFragment_Firmware(&SX1278,(uint8_t*) buffer_packet,(uint8_t*) buffer_flashing_data,
				&Local_Flag );
		/*Get CRC firmware*/
		local_u32CRC_fragment_firmware = (buffer_packet[8] << SHIFT_24_BIT)|(buffer_packet[7] << SHIFT_16_BIT)
										|(buffer_packet[6] << SHIFT_8_BIT)|(buffer_packet[5] << SHIFT_0_BIT);
		// Get RSSI && SNR
		if(Local_u16index_fragment == GW_SEND_DONE){
			BL_voidFinishBootLoader();
		}
//		gl_RSSI = SX1278_RSSI_LoRa(&SX1278);
//		gl_SNR  = SX1278_SNR(&SX1278);
		if(Local_u16index_fragment == 0){
			BL_voidSetConfigLoRa();
			LORA_IF_TransferData_Frame(&SX1278, (uint8_t*)  buffer_req, ret, MAX_TIME_OUT, SIZE_BUFFER_16BYTES, MCU_RECEIVED_CONFIG);
		}
		else if (local_totalPacket >0 && Local_u16index_fragment != 0 && Local_u16index_fragment <= Local_u16MaxPacket)
		{
						// CHECK CRC
			copy_Array_BL((uint8_t*) buffer_flashing_data ,(uint8_t*) buffer_packet, 64);
			if(BL_Check_CRC(local_u32CRC_fragment_firmware, (uint8_t*)buffer_flashing_data) == BL_OK){
				// Build bitMask for checking Lost Packet
				// set the position packet recieved
				setBit_BitMask(&bm,Local_u16index_fragment );
				Local_u32InactiveImageAddressCounter = Local_u32InactiveImageAddressCounter_Base +(Local_u16index_fragment-1)*64;
				//Encrypt packet
				//for(uint8_t local_counter_Encrypt = Local_u8index_fragment_previous ;local_counter_Encrypt< Local_u8index_fragment ;  local_counter_Encrypt++)
				//AES_CTR_xcrypt_buffer(&ctx_fw, (uint8_t*) buffer_flashing_data, 112);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
				for(uint8_t i = 0 ; i <8 ; i++){
					uint8_t byte_shift = (8*i);
					Local_u32HighByteDataReceive = (buffer_flashing_data[7+byte_shift] << SHIFT_24_BIT)  | (buffer_flashing_data[6+byte_shift] << SHIFT_16_BIT)
																	 | (buffer_flashing_data[5+byte_shift] << SHIFT_8_BIT)  | (buffer_flashing_data[4+byte_shift] << SHIFT_0_BIT);
					Local_u32LowByteDataReceive = (buffer_flashing_data[3+byte_shift] << SHIFT_24_BIT)  | (buffer_flashing_data[2+byte_shift] << SHIFT_16_BIT)
																	| (buffer_flashing_data[1+byte_shift] << SHIFT_8_BIT)  | (buffer_flashing_data[0+byte_shift] << SHIFT_0_BIT);
					HAL_FLASH_Unlock(); //Unlocks the flash memory
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Local_u32InactiveImageAddressCounter, Local_u32LowByteDataReceive);
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Local_u32InactiveImageAddressCounter + 4, Local_u32HighByteDataReceive);
					Local_u32InactiveImageAddressCounter+=8;
					HAL_FLASH_Lock();  //Locks again the flash memory
				}
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
				//Ping LED Flash when update
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				packet_current = local_totalPacket;
				//local_totalPacket -= 0x01;
				Local_u16index_fragment_previous = Local_u16index_fragment;
			}
						// get 128bytes in Flash
						// clear buffer packet
			memset((uint8_t*) buffer_packet, 0xff,80 );
			memset((uint8_t*) buffer_flashing_data, 0xff,64 );
		}
		if(Local_Flag == GW_SENDMEBITMAP){
			// encrypt packet send
			// add bitmask to buffer
			memset(buffer_mark_packet_loss , 0x00 , 80);
			buffer_mark_packet_loss[5] = local_totalPacket << SHIFT_8_BIT;
			buffer_mark_packet_loss[6] = local_totalPacket << SHIFT_0_BIT;
			for(uint32_t i =0 ; i < NUM_PACKETS_MAX/8 ; i++){
				buffer_mark_packet_loss[i+11] =bm.bitmask[i];
			}
			local_u32_CRC_bitmask = BL_Calculate_CRC(bm.bitmask , NUM_PACKETS_MAX);
			buffer_mark_packet_loss[10] = local_u32_CRC_bitmask >> SHIFT_24_BIT;
			buffer_mark_packet_loss[9] = local_u32_CRC_bitmask >> SHIFT_16_BIT;
			buffer_mark_packet_loss[8] = local_u32_CRC_bitmask >> SHIFT_8_BIT;
			buffer_mark_packet_loss[7] = local_u32_CRC_bitmask >> SHIFT_0_BIT;
			HAL_Delay(1000);
			LORA_IF_Stransmit_Response_Flashing(&SX1278, (uint8_t*) buffer_mark_packet_loss
					, ret, local_unicast_addr, MCU_REQUEST_PACKET_FW_LOSS);
		}


	}

}
void BL_voidFinishBootLoader(void)
{
	uint8_t Local_u8Count										= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32SizeActiveRegionRunning 					= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32CRCActiveRegionRunning 					= BL_INITIALIZE_WITH_ZERO;
	uint32_t local_unicast_addr 								= BL_INITIALIZE_WITH_ZERO;
	Local_u32SizeActiveRegionRunning = BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS);
	uint32_t Local_u32Appversion = BL_u32ReadAddressData(FLAG_STATUS_BANKSECOND_APP_VER_ADDRESS);
	Local_u32CRCActiveRegionRunning = BL_u32ReadAddressData(FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS);
	local_unicast_addr				= BL_Read_Address_Node();
	//Local_u32ActiveRegionRunning = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER , BL_RESET_BRANCHING_FLAG);
	HAL_Delay(100);
	// Verify Bank download Image
	if(BL_VerifyCheckSum(Local_u32SizeActiveRegionRunning, Local_u32CRCActiveRegionRunning, DOWNLOAD_BANK_START_ADDRESS) == BL_OK){
		//BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS , BANKFIRST_IMAGE);
			BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKSECOND_REGION_ADDRESS , BR_SET_IMAGE_ACTIVE);
			//Copy Bank download to bank 1
			BL_voidEraseBank(BANKFIRST_IMAGE);
			BL_voidCopyImageToActiveRegion();
			// update appversion
			BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_APP_VER_ADDRESS , Local_u32Appversion);
			BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS , BANKFIRST_IMAGE);
			//BL_voidMakeSoftWareReset();
			 NVIC_SystemReset();
	}
	else{
		LORA_IF_Stransmit_Response_Flashing(&SX1278, (uint8_t*) buffer_resp, ret, local_unicast_addr, MCU_ERROR_CRC);
		//BL_voidMakeSoftWareReset();
		NVIC_SystemReset();
	}
	/*Wait for User Reset*/
}

void BL_voidMakeSoftWareReset(void)
{
	// make software reset after flashing success
#ifdef Debug
	__HAL_DBGMCU_FREEZE_IWDG();
#endif
	 NVIC_SystemReset();
}
// Interrupt Received
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == DIO0_Pin){
		if(gl_State_BL == STATE_INITBOOT){
			if(LORA_IF_Stransmit_Request(&SX1278,(uint8_t*)  buffer_resp, ret, ADDR_NODE_1 , GW_SYNC_CONFIG) == LORA_OKE){
				HAL_NVIC_DisableIRQ(EXTI1_IRQn);
				//__HAL_GPIO_EXTI_CLEAR_IT(DIO0_Pin);
				// Change state
				gl_State_BL =STATE_RECEIVE_HEADER;

			}
		}
	}
}
//**************************Function Define***************************//

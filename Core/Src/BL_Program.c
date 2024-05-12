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

#define Debug
//**************************Include***************************//
#include "BL_Program.h"
#include "SX1278_if.h"
#include <string.h>
uint8_t buffer_req[16];
uint8_t buffer_flashing_data[64];
uint8_t buffer_mark_packet_loss[80];
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


void BL_voidBootLoader_Init(void)
{
	//BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER,BL_BRANCHING_FLAG_SET);
	// Read Branching Request Update Flag.
	// UI init boot
	uint32_t Local_u32Flag = BL_u32ReadAddressData(FLAG_STATUS_BOOTLOADER);
	if(Local_u32Flag == BL_BRANCHING_FLAG_RESET)
	{
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
		// Do nothing3
		// Application  not exited and backup image not exit
	}
}
uint8_t BL_CheckSize(uint32_t size_app){
	//Read size of first bank
	uint32_t Local_u32sizeofapp = BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS);

	return( Local_u32sizeofapp <= APP_ACTIVE_SIZE ) ? BL_OK : BL_SIZE_ERROR;
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
		uint32_t  Local_DataAddressVal  = BL_INITIALIZE_WITH_ZERO;
		uint32_t  Local_ActiveIndicateAddr  = BL_INITIALIZE_WITH_ZERO;
		// set the mark for the active addr
		Local_ActiveIndicateAddr = Active_Addr_Indicate;
		uint32_t Local_VectorTableOffset  = BL_INITIALIZE_WITH_ZERO;
		RCC->AHBENR |=0x40;
		u32Size_of_Image = u32Size_of_Image/4;
		/* Resets the CRC calculation unit and set the data register to 0xFFFF_FFFF*/
		CRC->CR = 0x01;
		for(uint32_t Local_u32Count = 0U ; Local_u32Count < u32Size_of_Image ; Local_u32Count++){
			/* Calculate CRC */
			Local_DataAddressVal = BL_u32ReadAddressData(Active_Addr_Indicate);
			if (Local_ActiveIndicateAddr  == BANKSECOND_IMAGE){
				Local_VectorTableOffset = Local_DataAddressVal - BANKSECOND_IMAGE;
				if(Local_VectorTableOffset > 0 &&Local_VectorTableOffset < 0xFFFF ){
					Local_DataAddressVal = BANKFIRST_IMAGE + Local_VectorTableOffset;
				}
			}
			CRC->DR  = Local_DataAddressVal;
			// Recall calculate for CRC IMG bank second
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
	Local_eraseInfo.NbPages =	32;

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
	    		if(Local_u32ActiveRegion != BR_SET_IMAGE_NOT_EXISTING)
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
	if(Local_u32ActiveImageStatus == BR_IMAGE_IS_ACTIVE  )
	{
		//Verify CheckSum
		if(BL_VerifyCheckSum(Local_u32SizeOfImageActive, Local_u32ReceivedCRC , SECOND_IMAGE_START_ADDRESS) == BL_OK ){
			if(Local_u32ActiveRegion != BR_SET_IMAGE_NOT_EXISTING)
				BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS, BANKSECOND_IMAGE);
			BL_voidJumpToActiveRegion();
		}
		else {
			BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_REGION_ADDRESS , BR_SET_IMAGE_CORRUPTED);
			//BL_voidMakeSoftWareReset();
		}

	}
	else if(Local_u32ActiveImageStatus == BR_IMAGE_IS_CORRUPTED || Local_u32ActiveImageStatus == BR_IMAGE_IS_NOT_EXISTING)
	{
		Local_u32BackupStatus    = BL_32CheckBankFirstRegion();
		// Check the status of the backup image if the image in he active region is corrupted or not exisit.
		if(Local_u32BackupStatus == BR_IMAGE_IS_CORRECT)
		{
			// update braching flag to bank first
			BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS , BANKFIRST_IMAGE);
			// Jump to Active bank
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

void BL_voidCheckActiveRegion(void)
{
    // Read Images Status To Determine Which Image Will Be Excuted.
	uint32_t Local_u32ActiveRegionRunning = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	switch(Local_u32ActiveRegionRunning)
	{
		case BANKFIRST_IMAGE:
			BL_voidCheckImgCorrectBankFirst();
			break;
		case BANKSECOND_IMAGE:
			BL_voidCheckImgCorrectBankSecond();
			break;
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
	SCB->VTOR = Local_u32ActiveImageAddress; //We relocate vector table to the sector 1 of Active Region
	__DSB(); //ARM says to use a DSB instruction just after 	relocating VTOR */

	AddressToCall();
}

void BL_voidJumpToBootloader(void)
{
	//@TODO: In develop
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
	BL_voidMakeSoftWareReset();
}


/*Update size of Image in header*/
void BL_voidUpdateHeaders(void)
{
	uint32_t Local_u32ImageSizeInBytes         = BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32CRCImage 					=BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32VerImage 					= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32ActiveRegionRunning = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	//Structure LoRa Transmit
	BL_voidSetConfigLoRa();
	// get Config Synchronize
	/* MCU response MCU_ENTER_FBL to GW and expect get resp as GW_SYNC_CONFIG*/
	LORA_IF_Stransmit_Response(&SX1278, buffer_resp, ret, ADDR_NODE_1, MCU_ACCEPT_REQUEST);
	ret = SX1278_LoRaEntryRx(&SX1278, SIZE_BUFFER_16BYTES , 10000);
	while(1){
		// Wait to get GW CONFIG
		// Request LoRa stransmit to get size of code
		if(LORA_IF_Stransmit_Request(&SX1278,(uint8_t*)  buffer_resp, ret, ADDR_NODE_1 , GW_SYNC_CONFIG ) == LORA_OKE)
		{
			//Send response to GW
			LORA_IF_TransferData_Frame(&SX1278, (uint8_t*)  buffer_req, ret, MAX_TIME_OUT, SIZE_BUFFER_16BYTES, MCU_RECEIVED_CONFIG);
			//Set Parameter and Configurate for LoRa
			u8SF= buffer_resp[3];
			u8BW= buffer_resp[4];
			u8CR= buffer_resp[5];
			/*Response Goto Programming and Send Update Request */
			SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, u8SF,
									u8BW, u8CR, SX1278_LORA_CRC_EN, 128);
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
				switch(Local_u32ActiveRegionRunning)
				{
				// update for bank first
				case BANKFIRST_IMAGE:

					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKSECOND_APP_VER_ADDRESS,Local_u32VerImage );
					//Set CRC
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS,Local_u32CRCImage);
					//set Bank Second IMG Corrupted - if flashing success update img correct
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKSECOND_REGION_ADDRESS,BR_SET_IMAGE_CORRUPTED);
					//update imgsize
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS,Local_u32ImageSizeInBytes);
					BL_voidEraseBank(BANKSECOND_IMAGE);
					break;
					// update for bank second
				case BANKSECOND_IMAGE:
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_APP_VER_ADDRESS,Local_u32VerImage );
					//Set CRC
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_CRC_BANKFIRST_REGION_ADDRESS,Local_u32CRCImage);
					//set Bank Second IMG Corrupted - if flashing success update img correct
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_REGION_ADDRESS,BR_SET_IMAGE_CORRUPTED);
					//update imgsize
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS,Local_u32ImageSizeInBytes);
					BL_voidEraseBank(BANKFIRST_IMAGE);
					break;
				case BR_SET_IMAGE_NOT_EXISTING :
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_APP_VER_ADDRESS,Local_u32VerImage );
					//Set CRC
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_CRC_BANKFIRST_REGION_ADDRESS,Local_u32CRCImage);
					//set Bank Second IMG Corrupted - if flashing success update img correct
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_REGION_ADDRESS,BR_SET_IMAGE_CORRUPTED);
					//update imgsize
					BL_voidEraseRestoreHeaderPage(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS,Local_u32ImageSizeInBytes);
					BL_voidEraseBank(BANKFIRST_IMAGE);
					break;
				default: break;
				}
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER , BL_RESET_BRANCHING_FLAG);
				BL_voidReceiveUpdate();
			}

//		else
//		{
//			LORA_IF_TransferData_Frame(&SX1278 , (uint8_t*) buffer_resp , ret , MAX_TIME_OUT ,SIZE_BUFFER_16BYTES ,MCU_CONFIG_ERROR);
//		}
		}
	//Reset SW
	//BL_voidMakeSoftWareReset();
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
	FLASH_EraseInitTypeDef Local_eraseInfo;
	uint32_t Local_u32ActiveRegionRunning                       = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	uint16_t Local_u16MaxPacket 								= BL_INITIALIZE_WITH_ZERO;
	uint16_t local_totalPacket									=BL_INITIALIZE_WITH_ZERO;
	switch (Local_u32ActiveRegionRunning) {
		case BANKFIRST_IMAGE:
			Local_u32SizeOfCode 								= BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS);
			Local_u32InactiveImageAddressCounter_Base               = SECOND_IMAGE_START_ADDRESS;
			Local_u32ExpectActiveAddr 								= SECOND_IMAGE_START_ADDRESS;
			break;
		case BANKSECOND_IMAGE:
			Local_u32SizeOfCode 								= BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS);
			Local_u32InactiveImageAddressCounter_Base               = FIRST_IMAGE_START_ADDRESS;
			Local_u32ExpectActiveAddr 								= FIRST_IMAGE_START_ADDRESS;
			break;
		case BR_SET_IMAGE_NOT_EXISTING :
			Local_u32SizeOfCode 								= BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS);
			Local_u32InactiveImageAddressCounter_Base               = FIRST_IMAGE_START_ADDRESS;
			Local_u32ExpectActiveAddr 								= FIRST_IMAGE_START_ADDRESS;
			break;
		default:
			break;
	}
	//Loop to receive code update
	memset((uint8_t*) buffer_mark_packet_loss ,0xff , 112 );
	local_totalPacket = (uint16_t)(Local_u32SizeOfCode/64)+1;
	Local_u16MaxPacket = local_totalPacket;
	// Init bitmask
	initBitMask(&bm);
	while(local_totalPacket>=0 && local_totalPacket != 0xFFFF )
	{
		/*Initial in the first time start up speed*/
		if(Local_u16index_fragment == 0)
			SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, u8SF,
												u8BW, u8CR, SX1278_LORA_CRC_EN, 128);
		AES_init_ctx_iv(&ctx_fw, AES_CBC_128_Key, AES_CBC_128_IV);
		memset((uint8_t*)buffer_packet , 0xff , 80);
		//Receive code update Fragment firmware
		Local_u16index_fragment = LORA_IF_GetFragment_Firmware(&SX1278,(uint8_t*) buffer_packet,(uint8_t*) buffer_flashing_data,
						ADDR_NODE_1 );
		// Get RSSI && SNR
		gl_RSSI = SX1278_RSSI_LoRa(&SX1278);
		gl_SNR  = SX1278_SNR(&SX1278);
		if(Local_u16index_fragment == 0){
			BL_voidSetConfigLoRa();
			LORA_IF_TransferData_Frame(&SX1278, (uint8_t*)  buffer_req, ret, MAX_TIME_OUT, SIZE_BUFFER_16BYTES, MCU_RECEIVED_CONFIG);
		}
		else if(Local_u16index_fragment == GW_SEND_DONE &&local_totalPacket >= 1){
			// encrypt packet send
			// add bitmask to buffer
			memset(buffer_mark_packet_loss , 0x00 , 80);
			buffer_mark_packet_loss[0] = ADDR_UNICAST ;
			buffer_mark_packet_loss[1] = ADDR_NODE_1;
			buffer_mark_packet_loss[3] = local_totalPacket << SHIFT_8_BIT;
			buffer_mark_packet_loss[4] = local_totalPacket << SHIFT_0_BIT;
			for(uint32_t i =0 ; i < NUM_PACKETS_MAX/8 ; i++){
				buffer_mark_packet_loss[i+16] =bm.bitmask[i];
			}
			LORA_IF_Stransmit_Response_Flashing(&SX1278, (uint8_t*) buffer_mark_packet_loss
					, ret, ADDR_NODE_1, MCU_REQUEST_PACKET_FW_LOSS);
		}
		else if (local_totalPacket >0 && Local_u16index_fragment != 0 && Local_u16index_fragment != GW_SEND_DONE && Local_u16index_fragment <= Local_u16MaxPacket && buffer_packet[2] == FL_FRAGMENT_FIRMWARE )
		{
				// Build bitMask for checking Lost Packet
				// set the position packet recieved
			    setBit_BitMask(&bm,Local_u16index_fragment );
			    Local_u32InactiveImageAddressCounter = Local_u32InactiveImageAddressCounter_Base +(Local_u16index_fragment-1)*64;
				copy_Array_BL((uint8_t*) buffer_flashing_data ,(uint8_t*) buffer_packet, 64);
				//Encrypt packet
				//for(uint8_t local_counter_Encrypt = Local_u8index_fragment_previous ;local_counter_Encrypt< Local_u8index_fragment ;  local_counter_Encrypt++)
				//AES_CTR_xcrypt_buffer(&ctx_fw, (uint8_t*) buffer_flashing_data, 112);
				for(uint8_t i = 0 ; i <8 ; i++){
					uint8_t byte_shift = (8*i);
					Local_u32HighByteDataReceive = (buffer_flashing_data[7+byte_shift] << SHIFT_24_BIT)  | (buffer_flashing_data[6+byte_shift] << SHIFT_16_BIT)
												 | (buffer_flashing_data[5+byte_shift] << SHIFT_8_BIT)  | (buffer_flashing_data[4+byte_shift] << SHIFT_0_BIT);
					Local_u32LowByteDataReceive = (buffer_flashing_data[3+byte_shift] << SHIFT_24_BIT)  | (buffer_flashing_data[2+byte_shift] << SHIFT_16_BIT)
												| (buffer_flashing_data[1+byte_shift] << SHIFT_8_BIT)  | (buffer_flashing_data[0+byte_shift] << SHIFT_0_BIT);
					//Set to rebuild vector table
					Local_u32OffsetVector_L = Local_u32LowByteDataReceive - BANKFIRST_IMAGE;
					Local_u32OffsetVector_H = Local_u32HighByteDataReceive - BANKFIRST_IMAGE;
					// Low Vector Addr
					if(Local_u32OffsetVector_L < 0xFFFF && Local_u32OffsetVector_L >0)
					{
						Local_u32LowByteDataReceive = Local_u32ExpectActiveAddr + Local_u32OffsetVector_L;
					}
					// High Vector Addr
					if(Local_u32OffsetVector_H < 0xFFFF && Local_u32OffsetVector_H >0)
					{
						Local_u32HighByteDataReceive = Local_u32ExpectActiveAddr + Local_u32OffsetVector_H;
					}
					HAL_FLASH_Unlock(); //Unlocks the flash memory
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Local_u32InactiveImageAddressCounter, Local_u32LowByteDataReceive);
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Local_u32InactiveImageAddressCounter + 4, Local_u32HighByteDataReceive);
					Local_u32InactiveImageAddressCounter+=8;
					HAL_FLASH_Lock();  //Locks again the flash memory

				}
				packet_current = local_totalPacket;
				local_totalPacket -= 0x01;

				// get 128bytes in Flash
				// clear buffer packet
				memset((uint8_t*) buffer_packet, 0xff,80 );
				memset((uint8_t*) buffer_flashing_data, 0xff,64 );
				Local_u16index_fragment_previous = Local_u16index_fragment;
				if(local_totalPacket == 0x00 && Local_u16counter_packet_loss ==0){
					BL_voidFinishBootLoader();
				}
		}

	}
}
void BL_voidFinishBootLoader(void)
{
	uint8_t Local_u8Count										= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32SizeActiveRegionRunning 					= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32CRCActiveRegionRunning 					= BL_INITIALIZE_WITH_ZERO;
	uint32_t Local_u32ActiveRegionRunning                       = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);

	switch (Local_u32ActiveRegionRunning)
	{
		case BANKFIRST_IMAGE:
			Local_u32SizeActiveRegionRunning = BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS);
			Local_u32CRCActiveRegionRunning =BL_u32ReadAddressData(FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS);
			Local_u32ActiveRegionRunning = BANKSECOND_IMAGE;
			break;
		case BANKSECOND_IMAGE:
			Local_u32SizeActiveRegionRunning = BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS);
			Local_u32CRCActiveRegionRunning =BL_u32ReadAddressData(FLAG_STATUS_CRC_BANKFIRST_REGION_ADDRESS);
			Local_u32ActiveRegionRunning = BANKFIRST_IMAGE;
			break;
		case BR_SET_IMAGE_NOT_EXISTING:
			Local_u32SizeActiveRegionRunning = BL_u32ReadAddressData(FLAG_STATUS_SIZE_BANKFIRST_REGION_ADDRESS);
			Local_u32CRCActiveRegionRunning =BL_u32ReadAddressData(FLAG_STATUS_CRC_BANKFIRST_REGION_ADDRESS);
			Local_u32ActiveRegionRunning = BANKFIRST_IMAGE;
			break;
		default:
			break;
	}
	//Local_u32ActiveRegionRunning = BL_u32ReadAddressData(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS);
	BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BOOTLOADER , BL_RESET_BRANCHING_FLAG);
	HAL_Delay(100);
	// Verify Indicate Image
	while(1){
		if(BL_VerifyCheckSum(Local_u32SizeActiveRegionRunning, Local_u32CRCActiveRegionRunning, Local_u32ActiveRegionRunning) == BL_OK){

			switch (Local_u32ActiveRegionRunning) {
			case BANKFIRST_IMAGE:
				BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS , BANKFIRST_IMAGE);
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKFIRST_REGION_ADDRESS , BR_SET_IMAGE_ACTIVE);
				break;
			case BANKSECOND_IMAGE:
				BL_voidEraseRestoreHeaderPage(FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS , BANKSECOND_IMAGE);
				BL_voidEraseRestoreHeaderPage(FLAG_STATUS_BANKSECOND_REGION_ADDRESS , BR_SET_IMAGE_ACTIVE);
				break;
			default:
				/*error*/
				break;
			}
			while(1){

				LORA_IF_Stransmit_Response_Flashing(&SX1278, (uint8_t*) buffer_packet , ret, ADDR_NODE_1, MCU_ACKNOWLEDGE_FINISHING);

	//			if(LORA_IF_GetFragment_Firmware(&SX1278,(uint8_t*) buffer_packet,(uint8_t*) buffer_flashing_data,
	//							ADDR_NODE_1 ) != GW_SEND_DONE )
	//			{
	//				break;
	//			}
				//Delay 100ms after send
				HAL_Delay(100);
				if(Local_u8Count == 10){
					BL_voidMakeSoftWareReset();
				}
				Local_u8Count++;
			}
		}

		else{
			LORA_IF_Stransmit_Response_Flashing(&SX1278, (uint8_t*) buffer_resp, ret, ADDR_NODE_1, MCU_ERROR_CRC);
		}
	}
	/*Wait for User Reset*/
}

void BL_voidMakeSoftWareReset(void)
{
	// make software reset after flashing success
#ifdef Debug
	__HAL_DBGMCU_FREEZE_IWDG();
#endif

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Reload = 9;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
}

//**************************Function Define***************************//

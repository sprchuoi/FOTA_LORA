/*
 * BL_Program.h
 *
 *  Created on: Mar 25, 2024
 *      Author: quang
 */

#ifndef INC_BL_PROGRAM_H_
#define INC_BL_PROGRAM_H_
//**************************Include***************************//
#include "main.h"
//**************************Include***************************//

//*************************Define****************************//
/** Check application checksum on startup */

#define USE_CHECKSUM 1

#define BOOTLOADER_IMAGE			(uint32_t)(0x08000000)
#define START_FLASH_MEMORY			BOOTLOADER_IMAGE
#define BANKFIRST_IMAGE              	(uint32_t)(0x08005000)      // Origin + Bootloader size (20kB)
#define FIRST_IMAGE_START_ADDRESS	BANKFIRST_IMAGE
#define BANKSECOND_IMAGE              	(uint32_t)(0x08010000)      // Origin + Bootloader size (20kB) + Active Bank (32kB)
#define DOWNLOAD_BANK_START_ADDRESS	BANKSECOND_IMAGE
#define FLASH_BANK_SIZE         	(0XB000)          // 44KB
#define FLASH_PAGE_SIZE_USER    	(0x400)           // 1kB
#define FLASH_BANK_NUMOFPAGE        (44)          	  // 44 Page
#define APP_ACTIVE_SIZE				(0XB000)	      // 44KB
#define SENSOR_ADDRESS_MAC 			(0x26011DEF)
//#define SENSOR_ADDRESS_MAC 			(0x260120F0)
//#define SENSOR_ADDRESS_MAC 			(0x26011BCD)
#define FIRST_PAGE_NUMBER_IN_BANKFIRST_IMAGE_REGION    0x14U
#define LAST_PAGE_NUMBER_IN_BANKFIRST_IMAGE_REGION    0x3FU

#define FIRST_PAGE_NUMBER_IN_BANKSECOND_IMAGE_REGION    0x3CU
#define LAST_PAGE_NUMBER_IN_BANKSECOND_IMAGE_REGION     0x67U
#define SIZE_IN_WORD_PER_BANK		 	5623		//word = 4 byte // page = 256 word => bank = page * 256 = 22*256


//Vector Table Reg
#define SCB_VTOR_ADDRESS  (0xE000ED08)
#define SCB_VTOR         *((volatile u32 *)SCB_VTOR_ADDRESS)

//Flag status Image
#define NUMBER_OF_FLAGS				  20U
#define PAGE_NUMBER_IN_FLAG_REGION    127U
#define WORD_SIZE_IN_BYTE                     4U
#define START_OF_FLAG_REGION                  0x0801FC00
#define END_OF_FLAG_REGION                    0x0801FC50
#define ERASED_VALUE                          0xffffffff
#define FLAG_STATUS_BOOTLOADER                (START_OF_FLAG_REGION)
#define FLAG_IMAGE							   FLAG_STATUS_BOOTLOADER
#define  NUM_PACKETS_MAX 						0x2C0U //704 packet
//Status region Bank 1
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
//
//STATE BL
//INIT BOOTLOADER
#define STATE_INITBOOT 				0x01U
#define STATE_RECEIVE_HEADER 		0x02U
#define STATE_START_FLASH 		    0x03U
#define STATE_RECEIVE_FW			0x04U
#define STATE_SEND_BIT_MAP			0x05U
#define STATE_SEND_DONE				0x06U

//Status branching define FW active
#define FLAG_INDICATE_ACTIVE_IMAGE_ADDRESS							(0x0801FC44)
#define FLAG_INDICATE_ADDRESS_NODE									(0x0801FC04)
// Branch Switch Constants
#define BR_SHIFT_LEFT_24_BIT                        24U
#define BR_SHIFT_LEFT_16_BIT                        16U
#define BR_SHIFT_LEFT_8_BIT                         8U
#define BR_ACTIVE_IMAGE_ID                          1U
#define BR_BACKUP_IMAGE_ID                          2U
#define BR_SET_IMAGE_NOT_EXISTING                   0xFFFFFFFF
#define BR_SET_IMAGE_ACTIVE                         0xFFFFFFF1
#define BR_SET_IMAGE_CORRECT                        0xFFFFFFF2
#define BR_SET_IMAGE_BACKUP                         0xFFFFFFF3
#define BR_SET_IMAGE_CORRUPTED                      0xFFFFFFF4
#define BR_IMAGE_IS_NOT_EXISTING                    0xFFFFFFFF
#define BR_IMAGE_IS_ACTIVE                          0xFFFFFFF1
#define BR_IMAGE_IS_CORRECT                         0xFFFFFFF2
#define BR_IMAGE_IS_BACKUP                          0xFFFFFFF3
#define BR_IMAGE_IS_CORRUPTED                       0xFFFFFFF4
#define BR_IMAGES_NOT_EXISTING_OR_CORRUPTTED        1
#define BR_IMAGES_EXISTING                          0

// BootLoader Constants
#define BL_INCREASE_RECORD_ADDRESS_WITH_16_BYTE     0x10U
#define BL_NUMBER_OF_DIGITS_IN_RECORD               35U
#define BL_THE_LAST_DIGIT_IN_RECORD                 '\n'
#define BL_RECEIVED_DATA                            1U
#define BL_RESET_COUNTER_TO_START_NEW_REC           0U
#define BL_NOT_RECEIVED                             0U
#define BL_BRANCHING_FLAG_SET                       0x00000000
#define BL_BRANCHING_FLAG_RESET                     0xFFFFFFFF
#define BL_RESET_BRANCHING_FLAG                     0xFFFFFFFF
#define BL_SET_BRANCHING_FLAG                       0x00000000
#define BL_INITIALIZE_WITH_FALSE                    0U
#define BL_INITIALIZE_WITH_ZERO                     0U
#define BL_INITIALIZE_EITH_CORRUPTED                0xFFFFFFF3

// declare bootloader error code

enum eBootloaderErrorCodes{
    BL_OK = 0,      /*!< No error */
    BL_NO_APP,      /*!< No application found in flash */
    BL_SIZE_ERROR,  /*!< New application is too large for flash */
    BL_CHKS_ERROR,  /*!< Application checksum error */
    BL_ERASE_ERROR, /*!< Flash erase error */
    BL_WRITE_ERROR, /*!< Flash write error */
    BL_OBP_ERROR    /*!< Flash option bytes programming error */
};


// define struct bitmask
 typedef struct {
     uint8_t bitmask[(NUM_PACKETS_MAX + 7) / 8]; // Sử dụng mảng bitmask để lưu trữ các bit
 } Bitmask;
// Sequence flash code define
#define GW_START_FLASHING                				0x01u
#define MCU_ACCEPT_REQUEST                          	0X05u
#define FL_FRAGMENT_FIRMWARE							0xfeu
#define MCU_ENTER_FBL									0x12u
#define GW_SYNC_CONFIG									0x20u
#define MCU_RECEIVED_CONFIG								0x21u
#define GW_PROVIDE_FW_INFO								0x30u
#define MCU_RECEIVED_SIZE_CODE							0x31u
#define MCU_REQUEST_PACKET 								0x32u
#define GW_START_SEND_FW								0x33u
#define MCU_CONFIG_SUCCESS         				        0x34u
#define MCU_CHECK_SECTOR_FINISHING                  	0x35u
#define MCU_ACCEPT_RECEIVING_PACKET_OF_CODE     		0x36u
#define MCU_ACKNOWLEDGE_LINE_OF_CODE_RECEIVED   		0x37u
#define MCU_WRITE_SUCCESS								0x38u
#define GW_SENDMEBITMAP  		                        0x39u
#define MCU_REQUEST_PACKET_FW_LOSS						0x76u
#define MCU_ACKNOWLEDGE_FINISHING               		0x77u
#define MCU_ACKNOWLEDGE_ACTIVE_CODE_CORRECT     		0x78u
#define MCU_ACKNOWLEDGE_ACTIVE_CODE_NOT_CORRECT 		0x79u
#define MCU_ACKNOWLEDGE_BACKUP_CODE_CORRECT     		0x7Au
#define MCU_ACKNOWLEDGE_BACKUP_CODE_NOT_CORRECT 		0x7Bu
#define GW_ACKNOWLEDGE_END_OTA                          0x7Cu
// Sequence Define Error
#define BL_LARGE_SIZE 									0x3Fu
#define MCU_ERROR_CRC									0x4Fu
#define BL_ERROR_KEY									0x5Fu
#define MCU_PACKET_INCORRECT							0x2Fu
#define MCU_IMAGE_CRC_NOT_CORRECT						0x8Fu
#define MCU_CONFIG_ERROR								0x7Fu

#define GW_SEND_DONE 									0xFFFCu

//Node ID define
#define ADDR_MASTER  0x23u
#define ADDR_NODE_1  0x01u
#define ADDR_NODE_2  0x02u
#define ADDR_BOARDCAST 0xFFu
#define ADDR_UNICAST 0x03u


#define HEADER_DATA_LENGTH 1U
#define RECEIVE_DATA_LENGTH 8U


//*************************Define****************************//

//*************************Structure****************************//
typedef void (*Application_t)(void);
//*************************Structure****************************//


//*************************Variable****************************//



//*************************Variable****************************//

//*************************Function****************************//

/******************************************************************************
* Function : BL_ReadAddressData(uint32_t address)
*//**
* \b Description:
*
* This function is used to check data at input address
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     uint32_t
*
* @return 		   uint32_t
*
* \b Example Example:
* @code
* 	BL_ReadAddressData(uint32_t address);
*
* @endcode
*
* @see BL_ReadAddressData(uint32_t address)
*
*******************************************************************************/
static uint32_t BL_u32ReadAddressData(uint32_t address);

/******************************************************************************
* Function : BL_voidSetConfigLoRa()
*//**
* \b Description:
*
* This function is used to check data at input address
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		   None
*
* \b Example Example:
* @code
* 	BL_voidSetConfigLoRa();
*
* @endcode
*
* @see BL_voidSetConfigLoRa()
*
*******************************************************************************/\
void BL_voidSetConfigLoRa();

/******************************************************************************
* Function : BL_voidBootLoader_Init()
*//**
* \b Description:
*
* This function is used to check flag to determine
* Executing Bootloader or Executing one of images.
*
* PRE-CONDITION:  HAL_Init , SystemClock_Config , HAL_GPIO_Init
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		   void
*
* \b Example Example:
* @code
* 	BL_voidBootLoader_Init();
*
* @endcode
*
* @see BL_voidBootLoader_Init
*
*******************************************************************************/
void BL_voidBootLoader_Init(void);

/******************************************************************************
* Function : BL_voidCheckActiveRegion()
*//**
* \b Description:
*
* This function is used to check images for executing or not.
*
* PRE-CONDITION:  FPEC Peripheral Initialized , HAL library .
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		   void
*
* \b Example Example:
* @code
* 	BL_voidCheckActiveRegion();
*
* @endcode
*
* @see BL_voidCheckActiveRegion
*
*******************************************************************************/
void BL_voidCheckActiveRegion(void);

/******************************************************************************
* Function : BL_32CheckBankFirstRegion()
*//**
* \b Description:
*
* This function is used to check images at the Bank First region
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		  uint32_t
*
* \b Example Example:
* @code
* 	BL_32CheckBankFirstRegion();
*
* @endcode
*
* @see BL_32CheckBankFirstRegion
*
*******************************************************************************/
uint32_t BL_32CheckBankFirstRegion(void);


/******************************************************************************
* Function : BL_32CheckBankSecondRegion()
*//**
* \b Description:
*
* This function is used to check images at the Bank First region
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		  uint32_t
*
* \b Example Example:
* @code
* 	BL_32CheckBankSecondRegion();
*
* @endcode
*
* @see BL_32CheckBankSecondRegion
*
*******************************************************************************/
uint32_t BL_32CheckBankSecondRegion(void);
/******************************************************************************
* Function :  BL_voidJumpToBootloader()
*//**
* \b Description:
*
* This function is used to jump to BootLoader region
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		   void
*
* \b Example Example:
* @code
* 	 BL_voidJumpToBootloader();
*
* @endcode
*
* @see  BL_voidJumpToBootloader
*
*******************************************************************************/



void BL_voidJumpToBootloader(void);

/******************************************************************************
* Function :  BL_voidJumpToActiveRegion()
*//**
* \b Description:
*
* This function is used to jump to Active region
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		  void
*
* \b Example Example:
* @code
* 	 BL_voidJumpToActiveRegion();
*
* @endcode
*
* @see  BL_voidJumpToActiveRegion
*
*******************************************************************************/
void BL_voidJumpToActiveRegion(void);

/******************************************************************************
* Function :  BL_voidCopyImageToActiveRegion()
*//**
* \b Description:
*
* This function is used to copy firmware backup region to active region
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     void
*
* @return 		  void
*
* \b Example Example:
* @code
* 	 BL_voidCopyImageToActiveRegion();
*
* @endcode
*
* @see  BL_voidCopyImageToActiveRegion
*
*******************************************************************************/
void BL_voidCopyImageToActiveRegion(void);

/******************************************************************************
* Function :  BL_voidCopyImageToBackupRegion(void)
*//**
* \b Description:
*
* This function is used to copy firmware active region to backup region
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidCopyImageToBackupRegion(void)
*
* @endcode
*
* @see  BL_voidCopyImageToBackupRegion
*
*******************************************************************************/
void BL_voidCopyImageToBackupRegion(void);

/******************************************************************************
* Function :  BL_voidEraseRestoreHeaderPage(uint32_t Copy_u32Address, uint32_t Copy_u32NewData)
*//**
* \b Description:
*
* This function is used to adjust flag
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     uint32_t Copy_u32Address
* 				  uint32_t Copy_u32NewData
*
* @return 		  void
*
* \b Example Example:
* @code
* 	BL_voidEraseRestoreHeaderPage(uint32_t Copy_u32Address, uint32_t Copy_u32NewData)
*
* @endcode
*
* @see  BL_voidEraseRestoreHeaderPage
*
*******************************************************************************/
void BL_voidEraseRestoreHeaderPage(uint32_t Copy_u32Address, uint32_t Copy_u32NewData);

/******************************************************************************
* Function :  BL_voidSetBranchingFlagAndMakeSWR(void)
*//**
* \b Description:
*
* This function is used to reset ECU after set flag
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidSetBranchingFlagAndMakeSWR(void)
*
* @endcode
*
* @see  BL_voidSetBranchingFlagAndMakeSWR
*
*******************************************************************************/
void BL_voidSetBranchingFlagAndMakeSWR(void);

/******************************************************************************
* Function :  BL_voidUpdateHeaders(void)
*//**
* \b Description:
*
* This function is used to receive size of new code and update flag
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidUpdateHeaders(void)
*
* @endcode
*
* @see  BL_voidUpdateHeaders
*
*******************************************************************************/
void BL_voidUpdateHeaders(void);

/******************************************************************************
* Function :  BL_voidReceiveUpdate(void)
*//**
* \b Description:
*
* This function is used to receive new firmware
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidReceiveUpdate(void)
*
* @endcode
*
* @see  BL_voidReceiveUpdate
*
*******************************************************************************/
void BL_voidReceiveUpdate(void);

/******************************************************************************
* Function :  BL_voidFinishBootLoader(void)
*//**
* \b Description:
*
* This function is used to finish boot_loader and reset HW then MCU will jump to application
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidFinishBootLoader(void)
*
* @endcode
*
* @see  BL_voidFinishBootLoader
*
*******************************************************************************/
void BL_voidFinishBootLoader(void);
/******************************************************************************
* Function : BL_CheckSize(uint32_t app_size)
*//**
* \b Description:
*
* This function is used to check the valid size code
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     uint32_t
*
* @return 		   uint32_t
*
* \b Example Example:
* @code
* 	BL_ReadAddressData(uint32_t address);
*
* @endcode
*
* @see BL_ReadAddressData(uint32_t address)
*
*******************************************************************************/
uint8_t BL_CheckSize(uint32_t app_size);

/******************************************************************************
* Function : BL_VerifyCheckSum(uint32_t app_size)
*//**
* \b Description:
*
* This function is used to check the checksum for ensure flashing success
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]     uint32_t
*
* @return 		   uint32_t
*
* \b Example Example:
* @code
* 	BL_ReadAddressData(uint32_t address);
*
* @endcode
*
* @see BL_ReadAddressData(uint32_t address)
*
*******************************************************************************/

uint8_t BL_VerifyCheckSum(uint32_t u32Length , uint32_t CRC_CODE , uint32_t Active_Addr_Indicate);

/******************************************************************************
* Function :  uint32_t *BL_GetFragmentFlash(uint32_t * u32Buffer_Flash, uint32_t Start_Addr)
*//**
* \b Description:
*
* This function is used to receive size of new code and update flag
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidUpdateHeaders(void)
*
* @endcode
*
* @see  BL_voidUpdateHeaders
*
*******************************************************************************/

void BL_GetFragmentFlash(uint32_t * u32Buffer_Flash, uint32_t Start_Addr);
/******************************************************************************
* Function :  BL_voidMakeSoftWareReset(void)
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_GetFragmentFlash(void)
*
* @endcode
*
* @see  BL_GetFragmentFlash
*
*******************************************************************************/
void BL_voidMakeSoftWareReset(void);
//*************************Function****************************//
void BL_voidCheckImgCorrectBankFirst(void);
/******************************************************************************
* Function :  BL_voidCheckImgCorrect(void)
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidCheckImgCorrectBankfirst(void)
*
* @endcode
*
* @see  BL_voidCheckImgCorrectBankfirst
*
*******************************************************************************/
void BL_voidCheckImgCorrectBankSecond(void);
/******************************************************************************
* Function :  BL_voidCheckImgCorrect(void)
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_voidCheckImgCorrectBankfirst(void)
*
* @endcode
*
* @see  BL_voidCheckImgCorrectBankfirst
*
*******************************************************************************/

HAL_StatusTypeDef BL_voidEraseBank(uint32_t Address_Flash);


/******************************************************************************
* Function :  initBitMask(void)
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	initBitMask(void)
*
* @endcode
*
* @see  initBitMask
*
*******************************************************************************/
void initBitMask(Bitmask *bm);


/******************************************************************************
* Function :  setBit_BitMask(Bitmask *bm , uint16_t packetnumber);
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	setBit_BitMask(Bitmask *bm , uint16_t packetnumber);
*
* @endcode
*
* @see  initBitMask
*
*******************************************************************************/

void setBit_BitMask(Bitmask *bm , uint16_t packetnumber);
/******************************************************************************
* Function :  setBit_BitMask(Bitmask *bm , uint16_t packetnumber);
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	setBit_BitMask(Bitmask *bm , uint16_t packetnumber);
*
* @endcode
*
* @see  initBitMask
*
*******************************************************************************/
uint8_t BL_Check_CRC(uint32_t CRC_expect , uint8_t *buffer_check);
/******************************************************************************
* Function :  BL_Read_Address_Node(Bitmask *bm , uint16_t packetnumber);
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_Read_Address_Node(Bitmask *bm , uint16_t packetnumber);
*
* @endcode
*
* @see  initBitMask
*
*******************************************************************************/
uint32_t BL_Read_Address_Node();
/******************************************************************************
* Function :  BL_Calculate_CRC(uint8_t *buffer , uint32_t lenght);
*//**
* \b Description:
*
* This function is used to Soft Reset
*
* PRE-CONDITION:  None
*
* POST-CONDITION: None
*
* @param [in]    void
*
* @return 		 void
*
* \b Example Example:
* @code
* 	BL_Calculate_CRC(uint8_t *buffer , uint32_t lenght);
*
* @endcode
*
* @see  initBitMask
*
*******************************************************************************/
uint32_t BL_Calculate_CRC(uint8_t *buffer , uint32_t lenght);
#endif /* INC_BL_PROGRAM_H_ */

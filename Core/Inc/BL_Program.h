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


#define BOOTLOADER_IMAGE			(0x08000000)
#define ACTIVE_IMAGE              	(0x08005000)      // Origin + Bootloader size (20kB)
#define ACTIVE_IMAGE_START_ADDRESS	ACTIVE_IMAGE
#define BACKUP_IMAGE              	(0x0800A800)      // Origin + Bootloader size (20kB) + Active Bank (22kB)
#define BACKUP_IMAGE_START_ADDRESS	BACKUP_IMAGE
#define FLASH_BANK_SIZE         	(0X5800)          // 22kB
#define FLASH_PAGE_SIZE_USER    	(0x400)           // 1kB
#define FLASH_BANK_NUMOFPAGE        (22)          	  // 22 Page


#define FIRST_PAGE_NUMBER_IN_ACTIVE_IMAGE_REGION    16
#define LAST_PAGE_NUMBER_IN_ACTIVE_IMAGE_REGION     37

#define FIRST_PAGE_NUMBER_IN_BACKUP_IMAGE_REGION    38
#define LAST_PAGE_NUMBER_IN_BACKUP_IMAGE_REGION     59
#define SIZE_IN_WORD_PER_BANK		 	5623		//word = 4 byte // page = 256 word => bank = page * 256 = 22*256


//Vector Table Reg
#define SCB_VTOR_ADDRESS  (0xE000ED08)
#define SCB_VTOR         *((volatile u32 *)SCB_VTOR_ADDRESS)

//Flag status Image
#define NUMBER_OF_FLAGS				  20
#define PAGE_NUMBER_IN_FLAG_REGION    127
#define WORD_SIZE_IN_BYTE                     4
#define START_OF_FLAG_REGION                  0x0801FC00
#define END_OF_FLAG_REGION                    0x0801FC50
#define ERASED_VALUE                          0xffffffff

#define FLAG_STATUS_BOOTLOADER                (START_OF_FLAG_REGION)
#define FLAG_IMAGE							   FLAG_STATUS_BOOTLOADER

#define FLAG_STATUS_ID_ADDRESS                                   (0x0801FC10)
#define FLAG_STATUS_ACTIVE_REGION_ADDRESS                        (0x0801FC14)
#define FLAG_STATUS_SIZE_ACTIVE_REGION_ADDRESS                   (0x0801FC18)
#define FLAG_STATUS_CRC_ACTIVE_REGION_ADDRESS                    (0x0801FC1C)
#define FLAG_STATUS_ENTRY_POINT_VALUE_ACTIVE_REGION_ADDRESS      (0x0801FC20)

#define FLAG_STATUS_BACKUP_IMAGE_ID_ADDRESS                      (0x0801FC30)
#define FLAG_STATUS_BACKUP_REGION_ADDRESS                 		 (0x0801FC34)
#define FLAG_STATUS_SIZE_BACKUP_REGION_ADDRESS                   (0x0801FC38)
#define FLAG_STATUS_CRC_BACKUP_REGION_ADDRESS                    (0x0801FC3C)
#define FLAG_STATUS_ENTRY_POINT_VALUE_BACKUP_REGION_ADDRESS      (0x0801FC40)

// Branch Switch Constants
#define BR_SHIFT_LEFT_24_BIT                        24
#define BR_SHIFT_LEFT_16_BIT                        16
#define BR_SHIFT_LEFT_8_BIT                         8
#define BR_ACTIVE_IMAGE_ID                          1
#define BR_BACKUP_IMAGE_ID                          2
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
#define BL_INCREASE_RECORD_ADDRESS_WITH_16_BYTE     0x10
#define BL_NUMBER_OF_DIGITS_IN_RECORD               35
#define BL_THE_LAST_DIGIT_IN_RECORD                 '\n'
#define BL_RECEIVED_DATA                            1
#define BL_RESET_COUNTER_TO_START_NEW_REC           0
#define BL_NOT_RECEIVED                             0
#define BL_BRANCHING_FLAG_RESET                     0xFFFFFFFF
#define BL_BRANCHING_FLAG_SET                       0x00000000
#define BL_RESET_BRANCHING_FLAG                     0xFFFFFFFF
#define BL_SET_BRANCHING_FLAG                       0x00000000
#define BL_INITIALIZE_WITH_FALSE                    0
#define BL_INITIALIZE_WITH_ZERO                     0
#define BL_INITIALIZE_EITH_CORRUPTED                0xFFFFFFF3

// Sequence flash code define
#define GW_FLASH_COMMAND                				0x01
#define MCU_ENTER_FLASHMODE                         	0X02
#define GW_PROVIDE_HEADER								0x30
#define MCU_RECEIVED_SIZE_CODE							0x31
#define MCU_REQUEST_PACKET 								0x32
#define MCU_ERASE_SECTOR                 				0x33
#define MCU_WRITE_SECTOR         						0x34
#define MCU_CHECK_SECTOR_FINISHING                  	0x35
#define MCU_ACCEPT_RECEIVING_PACKET_OF_CODE     		0x36
#define MCU_ACKNOWLEDGE_LINE_OF_CODE_RECEIVED   		0x37
#define GWY_ACKNOWLEDGE_FINISHING_SENDING_CODE  		0x38
#define MCU_ACKNOWLEDGE_FINISHING               		0x77
#define MCU_ACKNOWLEDGE_ACTIVE_CODE_CORRECT     		0x78
#define MCU_ACKNOWLEDGE_ACTIVE_CODE_NOT_CORRECT 		0x79
#define MCU_ACKNOWLEDGE_BACKUP_CODE_CORRECT     		0x7A
#define MCU_ACKNOWLEDGE_BACKUP_CODE_NOT_CORRECT 		0x7B
#define GW_ACKNOWLEDGE_FINISHING                        0x7C

//Node ID define
#define ADDR_MASTER  0x123
#define ADDR_NODE_1  0x001
#define ADDR_NODE_2  0x002


#define HEADER_DATA_LENGTH 1
#define RECEIVE_DATA_LENGTH 8

//Shift bit macro
#define SHIFT_24_BIT	24
#define SHIFT_16_BIT	16
#define SHIFT_8_BIT		8
#define SHIFT_0_BIT		0
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
* Function : BL_voidCheckActiveRegion()
*//**
* \b Description:
*
* This function is used to check images at the backup region
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
* 	BL_32CheckBackupRegion();
*
* @endcode
*
* @see BL_32CheckBackupRegion
*
*******************************************************************************/
uint32_t BL_32CheckBackupRegion(void);

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
* 	BL_voidMakeSoftWareReset(void)
*
* @endcode
*
* @see  BL_voidMakeSoftWareReset
*
*******************************************************************************/
void BL_voidMakeSoftWareReset(void);
//*************************Function****************************//

#endif /* INC_BL_PROGRAM_H_ */

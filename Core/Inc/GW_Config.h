/*
 * GW_Config : Using for save Configuration parameter for GW LoRa and General
 *
 * Data of create : 4/7/2024
 * Author : Binh NGuyen
 *
 * */
#ifndef _GW_CONFIG_H_
#define _GW_CONFIG_H_
#include "main.h"

//Define address
#define NUMBER_OF_FLAGS				    20
#define GW_START_OF_FLAG_ADDR 			(0x0801FC00)
#define GW_END_OF_FLAG_ADDR 			(0x0801FC50)
#define FLAG_STATUS_GW_CONFIG 			(GW_START_OF_FLAG_ADDR)
#define FLAG_PARAMETER_GW_CONFIG		(0x0801FC04)
#define FLAG_STATE_GW_CONFIG			(0x0801FC08)

//Define Value Flag
#define GW_CONFIG_PARAMETER_SET 		 0xFFFFFFFF
#define GW_CONFIG_PARAMETER_RESET		 0x00000000
#define GW_CONFIG_PARAMETER_SF_BW_CR_DEFAULT		 0x00010801 //  SF_9 BW07 CR4_5
#define ERASED_VALUE                      0xFFFFFFFF
#define WORD_SIZE_IN_BYTE                     4


//info flash
#define FLAG_STATUS_BANKSECOND_APP_VER_ADDRESS                      (0x0801FC30)
#define FLAG_STATUS_ADDRESS_TARGET_ADDRESS                 		    (0x0801FC34)
#define FLAG_STATUS_SIZE_BANKSECOND_REGION_ADDRESS                  (0x0801FC38)
#define FLAG_STATUS_CRC_BANKSECOND_REGION_ADDRESS                   (0x0801FC3C)
#define FLAG_STATUS_ENTRY_POINT_VALUE_BANKSECOND_REGION_ADDRESS     (0x0801FC40)
//parameter
extern uint8_t u8BW; // Bandwidth
extern uint8_t u8SF; // Spreading Frequency
extern uint8_t u8CR; // Coding Rate
extern uint8_t u8PW; // Power
extern uint8_t u8CHANNEL;// Channel
extern uint8_t u8CRC; // Enable/Disable CRC


/******************************************************************************
* Function : GW_Config_Init(void)
*//**
* \b Description:
*
* This function is used to Init Parameter Config for GW
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
* 	GW_Config_Init(void);
*
* @endcode
*
* @see GW_Config_Init(void)
*
*******************************************************************************/
void GW_Config_Init(void);
/******************************************************************************
* Function : GW_Config_SetParameter(void)
*//**
* \b Description:
*
* This function is used to get Paramter
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
* 	GW_Config_SetParameter
*
* @endcode
*
* @see GW_Config_SetParameter
*
*******************************************************************************/
uint32_t GW_Config_GetParameter(uint32_t Address);
/******************************************************************************
* Function : GW_voidEraseRestoreConfigPage(void)
*//**
* \b Description:
*
* This function is used to Erase and Restore Config Page
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
* 	GW_voidEraseRestoreConfigPage
*
* @endcode
*
* @see GW_voidEraseRestoreConfigPage
*
*******************************************************************************/

void GW_voidEraseRestoreConfigPage(uint32_t Copy_u32Address, uint32_t Copy_u32NewData);


/******************************************************************************
* Function : GW_Config_SetUp(void)
*//**
* \b Description:
*
* This function is used to set the Configuration of FW
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
* 	GW_Config_SetUp
*
* @endcode
*
* @see GW_Config_SetUp
*
*******************************************************************************/

void GW_Config_SetUp(void);





void GW_State_Init(void);

void GW_State_Save_State(uint8_t State);

void GW_Reset_State();
#endif /*_GW_CONFIG_H_*/

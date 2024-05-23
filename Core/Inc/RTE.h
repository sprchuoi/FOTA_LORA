/*
 * RTE.h
 *
 *  Created on: Apr 27, 2024
 *      Author: quang
 */

#ifndef INC_RTE_H_
#define INC_RTE_H_
/*User Include*/
#include "RTE_STD_Types.h"

#define   NULL_PTR      ((void *) 0)
#define INITIAL_VALUE_ZERO 0x00U
#define RTE_E_OKE 0x01U
#define RTE_E_INVALID 0x00U

// Port status Macro
#define IDLE 0x0U
#define BUSY 0x1U

typedef uint8_t SystemState;

/**************************************************************************/
/*                         Ports Write SystemState                         */
/**************************************************************************/
Std_ReturnType Rte_SystemState_WriteData(uint8_t SystemStateVar);
/**************************************************************************/
/*                         Ports Read SystemState                        */
/**************************************************************************/
Std_ReturnType Rte_SystemState_ReadData(uint8_t *SystemStateVar);
/**************************************************************************/
/*                         Ports Write Encrypt                            */
/**************************************************************************/
Std_ReturnType Rte_EncyptedBuffer_WriteData(uint8_t *EncyptedBufferP2Var);
Std_ReturnType Rte_EncyptedBufferFlag_WriteData(uint8_t SystemStateVar);
/**************************************************************************/
/*                         Ports Read Encrypt                             */
/**************************************************************************/
Std_ReturnType Rte_EncyptedBuffer_ReadData(uint8_t **EncyptedBufferP2Var);
Std_ReturnType Rte_EncyptedBufferFlag_ReadData(uint8_t *SystemStateVar);




/**************************************************************************/
/*                         Ports Write Update FW                         */
/**************************************************************************/
Std_ReturnType Rte_Crc_WriteData(uint32_t CRCVar);
Std_ReturnType Rte_ApplVer_WriteData(uint16_t ApplVerVar);
Std_ReturnType Rte_CodeSize_WriteData(uint32_t CodeSizeVar);
Std_ReturnType Rte_NodeAddr_WriteData(uint32_t NodeAddrVar);
Std_ReturnType Rte_HeaderAckFlag_WriteData(uint8_t HeaderAckFLagVar);
Std_ReturnType Rte_UpdateProgress_WriteData(uint8_t UpdateProgressVar);

/**************************************************************************/
/*                         Ports Read Receive Update FW                   */
/**************************************************************************/
Std_ReturnType Rte_Crc_ReadData(uint32_t *CRCVar);
Std_ReturnType Rte_ApplVer_ReadData(uint16_t *ApplVerVar);
Std_ReturnType Rte_CodeSize_ReadData(uint32_t *CodeSizeVar);
Std_ReturnType Rte_NodeAddr_ReadData(uint32_t *NodeAddrVar);
Std_ReturnType Rte_HeaderAckFlag_ReadData(uint8_t *HeaderAckFLagVar);
Std_ReturnType Rte_UpdateProgress_ReadData(uint8_t *UpdateProgressVar);


/**************************************************************************/
/*                        Ports  Write UI						 		  */
/**************************************************************************/
Std_ReturnType Rte_UserResponse_WriteData(uint8_t UserResponseVar);
Std_ReturnType Rte_UI_Error_WriteData(uint8_t UIErrorVar);

/**************************************************************************/
/*                        Ports  READ UI						 		  */
/**************************************************************************/

Std_ReturnType Rte_UI_Error_ReadData(uint8_t *UIErrorVar);
Std_ReturnType Rte_UserResponse_ReadData(uint8_t *UserResponseVar);
/**************************************************************************/
/*                        Ports  Write num packet FW Lora		 		  */
/**************************************************************************/
Std_ReturnType Rte_PacketSendLoraNum_WriteData(uint16_t NumPacketLoraFWVar);
Std_ReturnType Rte_FlagLoRaResp_WriteData(uint8_t FlagLoRaRespVar);
Std_ReturnType Rte_FlagConfigLoRA_WriteData(uint32_t FlagLoraConfigVar);
Std_ReturnType Rte_Reconstruct_VTOR_WriteData(uint8_t Active_AddressVar);

/**************************************************************************/
/*                        Ports  Read num packet FW Lora		 		  */
/**************************************************************************/
Std_ReturnType Rte_PacketSendLoraNum_ReadData(uint16_t *NumPacketLoraFWVar);
Std_ReturnType Rte_FlagLoRaResp_ReadData(uint8_t *FlagLoRaRespVar);
Std_ReturnType Rte_FlagConfigLoRA_ReadData(uint32_t *FlagLoraConfigVar);
Std_ReturnType Rte_Reconstruct_VTOR_ReadData(uint8_t *Active_AddressVar);


/**************************************************************************/
/*                         Ports Write Received Packet LoRa               */
/**************************************************************************/
Std_ReturnType Rte_PacketLoRaReceived_WriteData(uint8_t *BufferLoRaReceivedP2Var );
/**************************************************************************/
/*                         Ports Read Received Packet LoRa                */
/**************************************************************************/
Std_ReturnType Rte_PacketLoRaReceived_ReadData(uint8_t **BufferLoRaReceivedP2Var );



Std_ReturnType Rte_Flag_LoRaRequestDevice_WriteData(uint8_t FlagLoRa_DeviceVar);


Std_ReturnType Rte_Flag_LoRaRequestDevice_ReadData(uint8_t *FlagLoRa_DeviceVar);

#endif /* INC_RTE_H_ */

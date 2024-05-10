/*
 * RTE.c
 *
 *  Created on: Apr 27, 2024
 *      Author: quang
 */


#include "Rte.h"
#include "STD_TYPE.h"



/*RTE VAR GLOBAL*/
static uint8_t  gl_u8SystemState = INITIAL_VALUE_ZERO;
static uint32_t  gl_u32CRCVar = INITIAL_VALUE_ZERO;
static uint16_t  gl_u8ApplVerVar = INITIAL_VALUE_ZERO;
static uint32_t  gl_u32CodeSizeVar= INITIAL_VALUE_ZERO;
static uint8_t  gl_u8NodeAddrVar=INITIAL_VALUE_ZERO;
static uint8_t  gl_u8HeaderAckFLagVar = INITIAL_VALUE_ZERO;
static uint8_t  gl_u8UpdateProgressVar= INITIAL_VALUE_ZERO;
static uint8_t  gl_u8UserResponseVar = INITIAL_VALUE_ZERO;

static uint8_t  *gl_EncryptDataBufferPtr = NULL_PTR;
static uint8_t  gl_u8UIErrorVar = INITIAL_VALUE_ZERO;
static uint16_t  gl_u16NumPacketLoraFWVar = INITIAL_VALUE_ZERO;
static uint8_t  gl_u8FlagLoRaRespVar = INITIAL_VALUE_ZERO;
static uint8_t  gl_u8FlagLoraConfigVar = INITIAL_VALUE_ZERO;
static uint32_t gl_u32Active_AddressVar = INITIAL_VALUE_ZERO;

static uint8_t gl_u8EncyptedBufferFlagVar = INITIAL_VALUE_ZERO;

static uint8_t *gl_DataBufferLoRaPtr =  NULL_PTR;
/*Port*/
/**************************************************************************/
/*                         Ports Write SystemState                         */
/**************************************************************************/
Std_ReturnType Rte_SystemState_WriteData(uint8_t SystemStateVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8SystemState = SystemStateVar;
	return retVal;
}
/**************************************************************************/
/*                         Ports Read SystemState                        */
/**************************************************************************/
Std_ReturnType Rte_SystemState_ReadData(uint8_t *SystemStateVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*SystemStateVar) = gl_u8SystemState;
	return retVal;
}
/**************************************************************************/
/*                         Ports Write Encrypt                            */
/**************************************************************************/
Std_ReturnType Rte_EncyptedBuffer_WriteData(uint8_t *EncyptedBufferP2Var){
	Std_ReturnType retVal = RTE_E_OKE;
	if(EncyptedBufferP2Var != NULL_PTR)
		gl_EncryptDataBufferPtr = EncyptedBufferP2Var;
	return retVal;
}
Std_ReturnType Rte_EncyptedBufferFlag_WriteData(uint8_t EncyptedBufferFlagVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8EncyptedBufferFlagVar = EncyptedBufferFlagVar;
	return retVal;
}
/**************************************************************************/
/*                         Ports Read Encrypt                             */
/**************************************************************************/
Std_ReturnType Rte_EncyptedBuffer_ReadData(uint8_t **EncyptedBufferP2Var){
	Std_ReturnType retVal = RTE_E_OKE;
	if(EncyptedBufferP2Var != NULL_PTR)
		(*EncyptedBufferP2Var) = gl_EncryptDataBufferPtr;
	return retVal;
}
Std_ReturnType Rte_EncyptedBufferFlag_ReadData(uint8_t *EncyptedBufferFlagVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*EncyptedBufferFlagVar) = gl_u8EncyptedBufferFlagVar;
	return retVal;
}




/**************************************************************************/
/*                         Ports Write Update FW                         */
/**************************************************************************/
Std_ReturnType Rte_Crc_WriteData(uint32_t CRCVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u32CRCVar = CRCVar;
	return retVal;
}
Std_ReturnType Rte_ApplVer_WriteData(uint16_t ApplVerVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8ApplVerVar = ApplVerVar;
	return retVal;
}
Std_ReturnType Rte_CodeSize_WriteData(uint32_t CodeSizeVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u32CodeSizeVar = CodeSizeVar;
	return retVal;
}
Std_ReturnType Rte_NodeAddr_WriteData(uint8_t NodeAddrVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8NodeAddrVar = NodeAddrVar;
	return retVal;
}

Std_ReturnType Rte_HeaderAckFlag_WriteData(uint8_t HeaderAckFLagVar)
{
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8HeaderAckFLagVar = HeaderAckFLagVar;
	return retVal;
}
Std_ReturnType Rte_UpdateProgress_WriteData(uint8_t UpdateProgressVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8UpdateProgressVar = UpdateProgressVar;
	return retVal;
}

/**************************************************************************/
/*                         Ports Read Receive Update FW                   */
/**************************************************************************/
Std_ReturnType Rte_Crc_ReadData(uint32_t *CRCVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*CRCVar) = gl_u32CRCVar;
	return retVal;
}
Std_ReturnType Rte_ApplVer_ReadData(uint16_t *ApplVerVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*ApplVerVar) = gl_u8ApplVerVar;
	return retVal;
}
Std_ReturnType Rte_CodeSize_ReadData(uint32_t *CodeSizeVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*CodeSizeVar) = gl_u32CodeSizeVar;
	return retVal;
}
Std_ReturnType Rte_NodeAddr_ReadData(uint8_t *NodeAddrVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*NodeAddrVar) = gl_u8NodeAddrVar;
	return retVal;
}
Std_ReturnType Rte_HeaderAckFlag_ReadData(uint8_t *HeaderAckFLagVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*HeaderAckFLagVar) = gl_u8HeaderAckFLagVar;
	return retVal;
}
Std_ReturnType Rte_UpdateProgress_ReadData(uint8_t *UpdateProgressVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*UpdateProgressVar) = gl_u8UpdateProgressVar;
	return retVal;
}


/**************************************************************************/
/*                        Ports  Write UI						 		  */
/**************************************************************************/
Std_ReturnType Rte_UserResponse_WriteData(uint8_t UserResponseVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8UserResponseVar = UserResponseVar;
	return retVal;
}

Std_ReturnType Rte_UI_Error_WriteData(uint8_t UIErrorVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8UIErrorVar = UIErrorVar;
	return retVal;
}

/**************************************************************************/
/*                        Ports  Read UI						 		  */
/**************************************************************************/
Std_ReturnType Rte_UI_Error_ReadData(uint8_t *UIErrorVar ){
	Std_ReturnType retVal = RTE_E_OKE;
	(*UIErrorVar) = gl_u8UIErrorVar;
	return retVal;
}
Std_ReturnType Rte_UserResponse_ReadData(uint8_t *UserResponseVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*UserResponseVar) = gl_u8UserResponseVar;
	return retVal;
}
/**************************************************************************/
/*                        Ports  Write num packet FW Lora		 		  */
/**************************************************************************/
Std_ReturnType Rte_PacketSendLoraNum_WriteData(uint16_t NumPacketLoraFWVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u16NumPacketLoraFWVar = NumPacketLoraFWVar;
	return retVal;
}

Std_ReturnType Rte_FlagLoRaResp_WriteData(uint8_t FlagLoRaRespVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8FlagLoRaRespVar = FlagLoRaRespVar;
	return retVal;
}

Std_ReturnType Rte_FlagConfigLoRA_WriteData(uint32_t FlagLoraConfigVar){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_u8FlagLoraConfigVar = FlagLoraConfigVar;
	return retVal;
}



/**************************************************************************/
/*                        Ports  Read num packet FW Lora		 		  */
/**************************************************************************/
Std_ReturnType Rte_PacketSendLoraNum_ReadData(uint16_t *NumPacketLoraFWVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*NumPacketLoraFWVar) = gl_u16NumPacketLoraFWVar;
	return retVal;
}
Std_ReturnType Rte_FlagLoRaResp_ReadData(uint8_t *FlagLoRaRespVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*FlagLoRaRespVar) = gl_u8FlagLoRaRespVar;
	return retVal;
}

Std_ReturnType Rte_FlagConfigLoRA_ReadData(uint32_t *FlagLoraConfigVar){
	Std_ReturnType retVal = RTE_E_OKE;
	(*FlagLoraConfigVar) =gl_u8FlagLoraConfigVar ;
	return retVal;
}

/**************************************************************************/
/*                         Ports Write Received Packet LoRa            */
/**************************************************************************/
Std_ReturnType Rte_PacketLoRaReceived_WriteData(uint8_t *BufferLoRaReceivedP2Var ){
	Std_ReturnType retVal = RTE_E_OKE;
	gl_DataBufferLoRaPtr= BufferLoRaReceivedP2Var;
	return retVal;
}

/**************************************************************************/
/*                         Ports Read Received Packet LoRa            */
/**************************************************************************/
Std_ReturnType Rte_PacketLoRaReceived_ReadData(uint8_t **BufferLoRaReceivedP2Var ){
	Std_ReturnType retVal = RTE_E_OKE;
	(*BufferLoRaReceivedP2Var) = gl_DataBufferLoRaPtr;
	return retVal;
}






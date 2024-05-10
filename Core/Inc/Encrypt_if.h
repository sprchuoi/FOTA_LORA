/*
 * Encrypt_if.h
 *
 *  Created on: Apr 28, 2024
 *      Author: quang
 */

#ifndef INC_ENCRYPT_IF_H_
#define INC_ENCRYPT_IF_H_
#include "aes.h"
#include "RTE_FlashingPort.h"
#include "RTE.h"



/*****************************************************************************************/
/*                                   Include headres                                     */
/*****************************************************************************************/
#define STORE_AREA_START_ADDRESS 0x0800D000
#define ADDRESS_BANK_FIRST (uint32_t)(0x08005000)
#define ADDRESS_BANK_SECOND (uint32_t)(0x0800D000)
extern struct AES_ctx ctx;
/************************************************************************************

*Name       :   Encrypt_MainFunction

*Description: * read data from rom 16 byte in each round
              * encryp this buffer by AES module to cypher Data

*Pre-Cond   :  buffer flag is not set & sys flag is assign to encrypt

*pos-Cond   :  buffer flag is set & sys flag is assign to send

*Input      :   void

*Output     :   void

*Return     :   void

****************************************************************************************/

void Encrypt_MainFunc(void);

/************************************************************************************

*Name       :   Encrypt_MainFunc

*Description: Init Read address


*Pre-Cond   :  buffer flag is not set & sys flag is assign to encrypt

*pos-Cond   :  buffer flag is set & sys flag is assign to send

*Input      :   void

*Output     :   void

*Return     :   void

****************************************************************************************/
void Encrypt_Address_Read_Init(void) ;


#endif /* INC_ENCRYPT_IF_H_ */

/* RTE_STD_Types.h
 *
 *
 *  Created on: Apr 27, 2024
 *      Author: quang
 */

#ifndef INC_RTE_STD_TYPES_H_
#define INC_RTE_STD_TYPES_H_
/*User Include*/
#include "main.h"

typedef uint8_t PortStateType;
typedef uint8_t Flash_StateType;
typedef uint8_t Std_ReturnType;

// System State Macro
#define    WAIT_FOR_ESP_CONNECT		   0xaU
#define    SYS_IDLE                    0x0u
#define    SYS_NEW_UPDATE_REQ          0x1u
#define    SYS_RECEIVE_UPDATE          0x2u
#define    SYS_REQUEST_OTA			   0x3u
#define    SYS_WAIT_ACCEPT_OTA		   0x4u
#define    SYS_CONFIG_LORA			   0x5u
#define    SYS_ENCRYPT_FW              0x6u
#define    SYS_SEND_UPDATE             0x7u
#define    SYS_DONE_UPDATE             0x8u

// FLag Macro Buffer && Header && User resp
/* Buffer Flag values */
#define    BUFFER_NOT_SET              0x00u
#define    BUFFER_SET                  0x01u
#define    DATA_BUFFER_SIZE            0x10u
/* User response */
#define   ACCEPT_UPDATE                0X01u
#define   REFUSE_UPDATE                0X02u

#define   SHIFT_8_BIT					8U
#define   SHIFT_0_BIT					0U
#define   SHIFT_16_BIT					16U
#define   SHIFT_24_BIT					24U
#define   SHIFT_32_BIT					32U
#define   SHIFT_64_BIT					64U
#endif /* INC_RTE_STD_TYPES_H_ */

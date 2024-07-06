/*
 * operation.h
 *
 *  Created on: Mar 28, 2024
 *      Author: quang
 */

#ifndef INC_OPERATION_H_
#define INC_OPERATION_H_

#include "STD_TYPE.h"
#define BL_CONFIG 1
//#define GW_CONFIG 1
typedef enum{
	DONE=1,
	NOTDONE =0,
	EXCEPTION =-1
}Return_Operation_t;
// Copy Array
#if defined(GW_CONFIG) && GW_CONFIG
	void copy_Array(uint8* a , uint8* b , uint32 size);
#elif defined(BL_CONFIG) && BL_CONFIG
	void copy_Array_BL(uint8* a , uint8* b , uint32 size);
#endif
Return_Operation_t clearDataBuffer(uint8 *buffer , uint32 buffer_size);

void ConvertUInt32ToHexString(char* hexString);

uint32 ConvertArr32ToUint32(uint8 * buffer);
#endif /* INC_OPERATION_H_ */

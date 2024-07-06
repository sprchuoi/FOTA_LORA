/*
 * operation.c
 *
 *  Created on: Mar 28, 2024
 *      Author: quang
 */

#include "operation.h"
#include "string.h"
void copy_Array(uint8* a , uint8* b , uint32 size){
	// Start Position is the 4th of a
	uint8* prtA = a + 4 ;
	for(uint8 i = 0 ; i < size ; i++){
		*(prtA++) = *(b++);
	}
}
void copy_Array_BL(uint8* a , uint8* b , uint32 size){
	uint8* prtB = b + 4 ;
	for(uint8 i = 0 ; i < size ; i++){
		*(a++) = *(prtB++);
	}
}

Return_Operation_t clearDataBuffer(uint8 *buffer , uint32 buffer_size){
	// Clear buffer size
	memset(buffer, 0xFF, buffer_size);
	return DONE;
}


void ConvertUInt32ToHexString( char* hexString){
	uint32 imageSize = 255;
	sprintf(hexString , "0x%08X" , imageSize);


}

uint32 ConvertArr32ToUint32(uint8 * buffer){
	uint32 Data;
	Data = (buffer[0] >> SHIFT_0_BIT) & 0xFF;
	Data = (buffer[1] >> SHIFT_8_BIT) & 0xFF;
	Data = (buffer[2] >> SHIFT_16_BIT) & 0xFF;
	Data = (buffer[3] >> SHIFT_24_BIT) & 0xFF;
	return Data;
}



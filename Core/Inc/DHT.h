/*
 * DHT.h
 *
 *  Created on: May 31, 2024
 *      Author: quang
 */

#ifndef INC_DHT_H_
#define INC_DHT_H_
#include "main.h"
void microDelay(uint16_t delay);
uint8_t DHT_Read(void);
uint8_t DHT_Start(void);
#endif /* INC_DHT_H_ */

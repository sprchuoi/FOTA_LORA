/*
 * UserInterface.h
 *
 *  Created on: May 1, 2024
 *      Author: quang
 */

#ifndef INC_USERINTERFACE_H_
#define INC_USERINTERFACE_H_
#include "main.h"
void UI_InitBoot(void);
void UI_DisplayInformation(void);
void UI_Display_DataValue(uint8_t * buffer_DHT ,uint16_t MQ2_Val);
#endif /* INC_USERINTERFACE_H_ */

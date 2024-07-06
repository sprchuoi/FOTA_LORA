/*
 * UserInterface.c
 *
 *  Created on: May 1, 2024
 *      Author: quang
 */


#include "UserInterface.h"
#include "function.h"
#include "SSD1306.h"
static uint32_t gl_appver;
static uint32_t ActiveRegionAddress;
static uint32_t Node_ID;
/**********************************************
 * FUNCTION
 * ********************************************/
void UI_InitBoot(void){
	SSD1306_Clear();
	Node_ID = FUNC_ReaddataAddress(FLAG_NODE_ID);
	char buffer_data_id[4];
	sprintf(buffer_data_id, "%X", Node_ID);
	SSD1306_Init();
	SSD1306_GotoXY (0,10); // goto 10, 10
	SSD1306_Puts("FUOTA INIT", &Font_11x18, 1); // print start FUOTA
	SSD1306_GotoXY (5, 40);
	SSD1306_Puts ("Updating!!", &Font_7x10, 1);
	SSD1306_GotoXY (5, 50);
	SSD1306_Puts ("NODE ID:", &Font_7x10, 1);
	SSD1306_GotoXY (60, 50);
	SSD1306_Puts (buffer_data_id, &Font_7x10, 1);
	SSD1306_UpdateScreen(); // update screen
}
void UI_DisplayInformation(void){
	SSD1306_Init();
	gl_appver = FUNC_ReaddataAddress(FLAG_STATUS_BANKFIRST_APP_VER_ADDRESS);
	Node_ID = FUNC_ReaddataAddress(FLAG_NODE_ID);

	// Extract the major and minor version numbers from gl_appver
	uint8_t major_version = (gl_appver >> 8) & 0xFF; // Extract major version (e.g., 1 from 0x00000131)
	uint8_t minor_version = gl_appver & 0xFF; // Extract minor version (e.g., 3 from 0x00000131)

	char buffer_version[10];
	char buffer_data_id[10];
	sprintf(buffer_version, "%d.%d", major_version, minor_version);
	sprintf(buffer_data_id, "%X", Node_ID);

	SSD1306_GotoXY(5, 10);
	SSD1306_Puts("Version:", &Font_7x10, 1);
	SSD1306_GotoXY(60, 10);
	SSD1306_Puts(buffer_version, &Font_7x10, 1);
	SSD1306_GotoXY(5, 20);
	SSD1306_Puts("Node ID:", &Font_7x10, 1);
	SSD1306_GotoXY(60, 20);
	SSD1306_Puts(buffer_data_id, &Font_7x10, 1);
	SSD1306_UpdateScreen(); // Update screen

}
void UI_Display_DataValue(uint8_t *buffer_DHT ,uint16_t MQ2_Val){
	uint8_t  buffer_DHT_TempH = buffer_DHT[0] ;
	uint8_t  buffer_DHT_TempL = buffer_DHT[1] ;
	uint8_t  buffer_DHT_HumiH= buffer_DHT[2] ;
	uint8_t  buffer_DHT_HumiL= buffer_DHT[3] ;

	// Chuyển giá trị nhiệt độ và độ ẩm sang dạng chuỗi
	char temp_str[10];
	char humi_str[10];
	snprintf(temp_str, sizeof(temp_str), "%d.%dC", buffer_DHT_TempH, buffer_DHT_TempL);
	snprintf(humi_str, sizeof(humi_str), "%d.%d%%", buffer_DHT_HumiH, buffer_DHT_HumiL);
	// Chuyển giá trị MQ2 sang dạng chuỗi
	char mq2_str[10];
	snprintf(mq2_str, sizeof(mq2_str), "%u", MQ2_Val);
	SSD1306_GotoXY (5, 30);
	SSD1306_Puts ("Temp:", &Font_7x10, 1);
	SSD1306_GotoXY (40, 30);
	SSD1306_Puts (temp_str, &Font_7x10, 1);
	SSD1306_GotoXY (5, 40);
	SSD1306_Puts ("Humi:", &Font_7x10, 1);
	SSD1306_GotoXY (40, 40);
	SSD1306_Puts (humi_str, &Font_7x10, 1);
	//SSD1306_GotoXY (5, 50);
	//SSD1306_Puts ("MQ2:", &Font_7x10, 1);
	//SSD1306_GotoXY (40, 50);
	//SSD1306_Puts (mq2_str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

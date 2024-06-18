/*
 * UserInterface_privateFunc.h
 *
 *  Created on: Apr 27, 2024
 *      Author: quang
 */

#ifndef INC_USERINTERFACE_PRIVATEFUNC_H_
#define INC_USERINTERFACE_PRIVATEFUNC_H_
#include "main.h"
// Declare Macro state FUNC
// UI state
typedef uint8_t UIStateType;
// UI Cursor Position state
typedef uint8_t UIErrorType;

#define INIT_VAL_ZERO			  0U
// define UI UPDATE STATE
#define     UI_IDLE               0x00U
#define     UI_DOWNLOADING_FW     0x01U
#define 	UI_START_OTA		  0x02U
#define 	UI_WAIT_START_OTA	  0x03U
#define 	UI_SYNCONFIGURATION	  0x04U
#define     UI_SENDPACKET_LORA_FW 0x05U
#define		UI_RESENDPACKET_LORA_FW 0x06U
#define 	UI_DISPLAYERROR		   0x07U
#define 	UI_DONE_OTA           0X08U
#define 	UI_DOWNLOADING_FW     0x09U
#define 	UI_WAIT_FOR_WIFI	  0x0AU

// STATE ERROR
#define 	UI_ERROR_RESET 			0x01U
#define 	UI_ERROR_SET			0x00U
// PROGRESS ESTIMATE
#define INTER_PACKET_TIME 1.0
#define PACKET_SIZE 80
#define PROGRESS_BAR_WIDTH 100  // Chiều rộng của thanh tiến trình
// define Variable UI
#define     UI_SCREEN_WIDTH      128
#define     UI_SCREEN_HIGHT      160
#define     UI_BORDER_WIDTH      8

#define     UI_FIRST_LINE_X      10
#define     UI_SECOND_LINE_X     40
#define     UI_THIRD_LINE_X      80
#define     UI_FOURTH_LINE_X     120

#define     UI_FIRST_ROW_Y       10
#define     UI_SECOND_ROW_Y      30
#define     UI_THIRD_ROW_Y       60
#define     UI_FOURTH_ROW_Y      90

//#define     UI_TEXT_COLOR      TFT_RED
#define     UI_MAIN_TEXT_SIZE       3
#define     UI_SECONARY_TEXT_SIZE   2

/***** Buttons *****/
#define      BUTTON_TASK_PERIOD    20
#define      BUTTON_SWIPE_ID       0x01u
#define      BUTTON_OK_ID          0x00u

#define    WAIT_TIME_AFTER_DONE  2000


static void UI_Init_Flashing_Screen(void);
static void UI_Clean(void);
static void UI_Node_Connect(int nodeIndex, bool connected) ;
static void UI_WaitForResp(void);
static void UI_Downloading_FW(void);
static void UI_UpdateDownloading_FW(uint8_t Var_Progress);
static void UI_DoneDownload_FW(void);
static void UI_Synconfiguaraton(void);
static void UI_SendSW_LoRa(uint16_t Var_numPacket);
static void UI_DisplayERROR(uint8_t Var_UIError);
static void UI_StartOTA(void);
static void UI_Wait_ACCEPT_OTA(uint16_t Var_Time_request);
static void UI_Send_Packet_Lost(uint16_t Var_numPacket);
static void UI_DisplayConfig(uint8_t bw, uint8_t sf, uint8_t cr);
static void UI_Wait_ForConnect_WiFi();
static void UI_WiFi_Connected();
static float UI_CalculateLoRaDataRate(uint8_t sf, uint32_t bw, uint8_t cr);

#endif /* INC_USERINTERFACE_PRIVATEFUNC_H_ */

/*
 * UserInterface.c
 *
 *  Created on: Apr 27, 2024
 *      Author: quang
 */
#include "UserInterface.h"
#include "UserInterface_privateFunc.h"
#include "SSD1306.h"
#include "stdio.h"

/**************************************************/
/*               Global Variable                  */
/**************************************************/
static UIStateType  gl_UISTATE;
static SystemState gl_SystemState;
static UICursorStateType gl_UICURSORSTATE;
static uint8_t gl_u8DownLoadProgress ;
static uint8_t gl_u8ErrorFlag;
static uint16_t gl_u16Packet_LoRa_FW;
static uint32_t gl_u32SizeCodeFw;
static uint8_t gl_u8FlagLoRaPacket;
static uint8_t gl_counterdot;
static uint16_t gl_TimeRequest;
static uint32_t gl_u32Version;
/**************************************************/
/*				  Init FUNC					      */
/**************************************************/
void UI_Init(void){
	// Init variables
	gl_UISTATE = UI_START_OTA;
	gl_u8DownLoadProgress = INIT_VAL_ZERO;
	gl_u32SizeCodeFw = INIT_VAL_ZERO;
	gl_counterdot = INIT_VAL_ZERO;
	// Init OLED display
	SSD1306_Init();
	/*Run main Appication before go to Flashing Appl*/
	// Init Flashing
	UI_Init_Flashing_Screen();

}

void UI_Main_FLASHING(void){
	switch (gl_UISTATE) {
		/* UI_IDE STATE */
		case UI_IDLE:
		{
			// Get System state Via RTE
			Std_ReturnType retVal  = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			if(RTE_E_OKE == retVal){
				if(SYS_NEW_UPDATE_REQ == gl_SystemState )
				{
					// Clean UI screen
					UI_Clean();
					// Display Information for FLashing
					UI_WaitForResp();

					gl_UISTATE =  UI_DOWNLOADING_FW;
				}
			}
			else{
				/*State In IDLE STATE and Change UI to ERROR STATE*/
				gl_UISTATE = UI_DISPLAYERROR;
			}
			break;
		}
		case UI_DOWNLOADING_FW:
		{
			Std_ReturnType retVal = RTE_RUNNABLE_DOWNLOAD_PROGRESS_ReadData(&gl_u8DownLoadProgress);
			if( RTE_E_OKE == retVal){
				UI_UpdateDownloading_FW(gl_u8DownLoadProgress);
				if(100 == gl_u8DownLoadProgress)
				{
					HAL_Delay(2000);
					UI_Clean();
					UI_SendSW_LoRa(INITIAL_VALUE_ZERO);
					gl_UISTATE = UI_SYNCONFIGURATION;
				}
			}
			break;
		}
		case UI_START_OTA:
		{
			Std_ReturnType retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			if (RTE_E_OKE == retVal){
				if(gl_SystemState == SYS_REQUEST_OTA){
					UI_Clean();
					UI_StartOTA();
				}
				else {
					gl_UISTATE = UI_WAIT_START_OTA;
					//UI_Clean();
				}
			}
			break;
		}
		case UI_WAIT_START_OTA:
		{
			Std_ReturnType retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			retVal = Rte_PacketSendLoraNum_ReadData(&gl_TimeRequest);
			if(RTE_E_OKE == retVal){
				if(gl_SystemState == SYS_WAIT_ACCEPT_OTA){
					UI_Wait_ACCEPT_OTA(gl_TimeRequest);

				}
				else if(gl_SystemState == SYS_REQUEST_OTA){
					gl_UISTATE = UI_START_OTA;
				}
				else if (gl_SystemState == SYS_CONFIG_LORA){
					gl_UISTATE = UI_SYNCONFIGURATION;
					UI_Clean();
				}
				else {
					/*OUT OF REQUEST*/
					gl_UISTATE=UI_DISPLAYERROR;
					UI_Clean();
				}
			}

		}

		case UI_SYNCONFIGURATION:
		{	Std_ReturnType retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			retVal = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_u8ErrorFlag);
			if( RTE_E_OKE == retVal){
				if(gl_SystemState == SYS_CONFIG_LORA){
					UI_Synconfiguaraton();
				}
				else if(gl_SystemState == SYS_SEND_UPDATE)
					gl_UISTATE = UI_SENDPACKET_LORA_FW;
				if(gl_u8ErrorFlag == GW_REQUEST_UPDATE_ERROR){
					UI_Clean();
					gl_UISTATE =UI_DISPLAYERROR;
					UI_DisplayERROR(gl_u8ErrorFlag);
				}
			}
			break;
		}
		case UI_SENDPACKET_LORA_FW:
		{

			Std_ReturnType retVal_1 = RTE_RUNNABLE_PACKET_SEND_LORA_NUM_ReadData(&gl_u16Packet_LoRa_FW);

			if(RTE_E_OKE == retVal_1){
				UI_SendSW_LoRa(gl_u16Packet_LoRa_FW);
				Std_ReturnType retVal_2 = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_u8ErrorFlag);
				if( MCU_ERROR_CRC == gl_u8ErrorFlag  ){
					UI_Clean();
					gl_UISTATE = UI_DISPLAYERROR;
				}
				else if( MCU_ACKNOWLEDGE_FINISHING == gl_u8ErrorFlag){
					UI_Clean();
					UI_DoneDownload_FW();
					gl_UISTATE = UI_DONE_OTA;
				}
				else if(MCU_REQUEST_PACKET_FW_LOSS ==gl_u8ErrorFlag ){
					UI_Clean();
					gl_UISTATE = UI_RESENDPACKET_LORA_FW;
				}

			}
			break;
		}
		case UI_RESENDPACKET_LORA_FW:
		{
			Std_ReturnType retVal_1 = RTE_RUNNABLE_PACKET_SEND_LORA_NUM_ReadData(&gl_u16Packet_LoRa_FW);
			if(RTE_E_OKE == retVal_1){
				UI_Send_Packet_Lost(gl_u16Packet_LoRa_FW);
				Std_ReturnType retVal_2 = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_u8ErrorFlag);
				if(MCU_ERROR_CRC  ==gl_u8ErrorFlag){
					UI_Clean();
					gl_UISTATE = UI_DISPLAYERROR;
				}
				else if(MCU_ACKNOWLEDGE_FINISHING == gl_u8ErrorFlag){
					UI_Clean();
					UI_DoneDownload_FW();
					gl_UISTATE = UI_DONE_OTA;
				}
			}
			break;
		}
		case UI_DONE_OTA:
		{
			gl_UISTATE = UI_IDLE;
			UI_DoneDownload_FW();
			HAL_Delay(2000);
			UI_Clean();
			UI_Init_Flashing_Screen();
			break;
		}
		case UI_DISPLAYERROR:
		{
			/*ERROR Handler */
			Std_ReturnType retVal = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_u8ErrorFlag);
			if(RTE_E_OKE == retVal){
				UI_DisplayERROR(gl_u8ErrorFlag);

			}
			break;
		}
		default:
			/*ERROR */
			gl_UISTATE = UI_DISPLAYERROR;
			break;
	}
}
/**********************************************************/
/*					   Private UI FUNC					  */
/**********************************************************/
static void UI_Init_Flashing_Screen(void){
	/* Set Backgroun color */
	SSD1306_GotoXY (0,10); // goto 10, 10
	SSD1306_Puts("GATEWAY", &Font_11x18, 1); // print start FUOTA
	SSD1306_GotoXY (5, 40);
	SSD1306_Puts ("FUOTA UPDATE!!", &Font_7x10, 1);
	SSD1306_UpdateScreen(); // update screen
}
static void UI_Clean(void){
	SSD1306_Clear();
	SSD1306_UpdateScreen();
}

static void UI_WaitForResp(void){
	uint8_t local_u8NodeAddr = 0U;
	uint32_t local_u32Codesize = 0U;
	uint16_t local_u16Appver=0U;
	RTE_RUNNABLE_CODE_SIZE_ReadData(&local_u32Codesize);
	RTE_RUNNABLE_APP_VER_ReadData(&local_u16Appver);
	RTE_RUNNABLE_NODE_ADDR_ReadData(&local_u8NodeAddr);
	char Local_DataBuffer[4];
	uint8_t local_estimatime = local_u32Codesize/BandWidth_UART +20;
	SSD1306_GotoXY (20, 0);
	SSD1306_Puts ("Update Downloading!!", &Font_7x10, 1);
	sprintf(Local_DataBuffer , "%d" ,local_u16Appver );
	SSD1306_GotoXY (20, 10);
	SSD1306_Puts ("Version:", &Font_7x10, 1);
	SSD1306_GotoXY (30, 10);
	SSD1306_Puts (Local_DataBuffer, &Font_7x10, 1);
	SSD1306_GotoXY (20, 20);
	SSD1306_Puts ("Address:", &Font_7x10, 1);
	sprintf(Local_DataBuffer , "%d" ,local_u8NodeAddr );
	SSD1306_GotoXY (30, 20);
	SSD1306_Puts (Local_DataBuffer, &Font_7x10, 1);
	SSD1306_UpdateScreen(); // update screen
}
static void UI_Downloading_FW(void){
	/* Writeing Text */
	SSD1306_GotoXY (40, 10);
	SSD1306_Puts ("Download", &Font_7x10, 1);
	SSD1306_GotoXY (40, 20);
	SSD1306_Puts ("  in", &Font_7x10, 1);
	SSD1306_GotoXY (40, 30);
	SSD1306_Puts ("Progress", &Font_7x10, 1);
	SSD1306_GotoXY (50, 40);
	SSD1306_Puts ("  0%", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}
static void UI_UpdateDownloading_FW(uint8_t Var_Progress){
	char local_UpdateBuffer[4];
	sprintf(local_UpdateBuffer, "%d", Var_Progress);
	SSD1306_GotoXY (50, 40);
	SSD1306_Puts ("   %", &Font_7x10, 1);
	SSD1306_GotoXY (50, 40);
	SSD1306_Puts (local_UpdateBuffer, &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}
static void UI_DoneDownload_FW(void){
	/* Writing Text */
	Std_ReturnType retVal = RTE_RUNNABLE_APP_VER_ReadData(&gl_u32Version);
	char local_UpdateBuffer[4];
	sprintf(local_UpdateBuffer , "%d" , gl_u32Version);
	SSD1306_GotoXY (40, 10);
	SSD1306_Puts ("FUOTA", &Font_11x18, 1);
	SSD1306_GotoXY (30, 30);
	SSD1306_Puts ("COMPLETED", &Font_7x10, 1);
	SSD1306_GotoXY (30, 40);
	SSD1306_Puts ("VERSION: ", &Font_7x10, 1);
	SSD1306_GotoXY (100, 40);
	SSD1306_Puts (local_UpdateBuffer, &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}

static void UI_Synconfiguaraton(void){
	SSD1306_GotoXY (5, 20);
	SSD1306_Puts ("SYNC UP ", &Font_7x10, 1);
	SSD1306_Puts ("...", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}
static void UI_SendSW_LoRa(uint16_t Var_numPacket){
	char local_UpdateBuffer[4];
	sprintf(local_UpdateBuffer, "%d", Var_numPacket);
	SSD1306_GotoXY (5, 20);
	SSD1306_Puts ("UPDATING...", &Font_7x10, 1);
	SSD1306_GotoXY (5, 40);
	SSD1306_Puts ("PACKET SEND :", &Font_7x10, 1);
	SSD1306_GotoXY (100, 40);
	SSD1306_Puts (local_UpdateBuffer, &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}

static void UI_StartOTA(void){
	SSD1306_GotoXY (0,10); // goto 10, 10
	SSD1306_Puts("Send Request OTA!!", &Font_7x10, 1); // print start FUOTA
	SSD1306_UpdateScreen(); // update screen
}
static void UI_Wait_ACCEPT_OTA(uint16_t Var_Time_request){
	char local_DataBuffer[4];
	sprintf(local_DataBuffer, "%d", Var_Time_request);
	SSD1306_GotoXY (0, 40);
	SSD1306_Puts ("Time Request OTA:", &Font_7x10, 1);
	SSD1306_GotoXY (120, 40);
	SSD1306_Puts (local_DataBuffer, &Font_7x10, 1);
	SSD1306_UpdateScreen(); // update screen
}

static void UI_DisplayERROR(uint8_t Var_UIError){
	char local_ErrorBuffer[4];
	sprintf(local_ErrorBuffer, "%d", Var_UIError);
	SSD1306_GotoXY (50, 20);
	SSD1306_Puts ("ERROR :", &Font_7x10, 1);
	SSD1306_GotoXY (10, 40);
	switch (Var_UIError) {
		case MCU_ERROR_CRC:
			SSD1306_Puts ("INVALID CRC", &Font_7x10, 1);
			break;
		case GW_REQUEST_UPDATE_ERROR:
			SSD1306_Puts ("REQUEST UPDATE ERROR", &Font_7x10, 1);
			break;
		case GW_OUTOFREQUEST_ERROR:
			SSD1306_Puts ("REQUEST UPDATE FAIL", &Font_7x10, 1);
			break;
		default:
			SSD1306_Puts ("UNKNOWN ERROR!!", &Font_7x10, 1);
			break;
	}
	SSD1306_UpdateScreen(); //display
}

static void UI_Send_Packet_Lost(uint16_t Var_numPacket){
	char local_UpdateBuffer[4];
	sprintf(local_UpdateBuffer, "%d", Var_numPacket);
	SSD1306_GotoXY (5, 20);
	SSD1306_Puts ("UPDATING...", &Font_7x10, 1);
	SSD1306_GotoXY (5, 40);
	SSD1306_Puts ("PACKET RESENT:", &Font_7x10, 1);
	SSD1306_GotoXY (100, 40);
	SSD1306_Puts (local_UpdateBuffer, &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}

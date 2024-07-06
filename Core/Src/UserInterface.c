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
#include "math.h"

/**************************************************/
/*               Global Variable                  */
/**************************************************/
static UIStateType  gl_UISTATE;
static SystemState gl_SystemState;
static UIErrorType gl_UIERRORTYPE;
static uint8_t gl_u8DownLoadProgress ;
static uint8_t gl_u8ErrorFlag;
static uint16_t gl_u16Packet_LoRa_FW;
static uint32_t gl_u32SizeCodeFw;
static uint8_t gl_counterdot;
static uint16_t gl_TimeRequest;
static uint32_t gl_u32Version;
//static uint8_t gl_u8NodeIndex;
static uint16_t  TOTAL_PACKETS;
static uint8_t gl_u8NodeStatus_1;
static uint8_t gl_u8NodeStatus_2;
static uint8_t gl_u8NodeStatus_3;
static uint32_t gl_u32configLoRa;
static float data_rate;
static float transmission_time_per_packet;
static const char* SX1278_Bandwidth_UI[10] = { "7.8", "10.4", "15.6", "20.8", "31.2", "41.7", "62.5", "125", "250", "500" };
static const uint8_t SX1278_SpreadFactor_UI[7] = { 6, 7, 8, 9, 10, 11, 12 };
static const char* SX1278_CodingRate_UI[4] = { "4/5", "4/6", "4/7", "4/8" };
static const float SX1278_Bandwidth_Values[10] = { 7.8, 10.4, 15.6, 20.8, 31.2, 41.7, 62.5, 125, 250, 500 };
static const uint8_t SX1278_SpreadFactor_Values[7] = { 6, 7, 8, 9, 10, 11, 12 };
static const float SX1278_CodingRate_Values[4] = { 1, 2, 3, 4 };
/**************************************************/
/*				  Init FUNC					      */
/**************************************************/
void UI_Init(void){
	// Init variables
	gl_UISTATE = UI_IDLE;
	//gl_UISTATE = UI_START_OTA;
	gl_u8DownLoadProgress = INIT_VAL_ZERO;
	gl_u32SizeCodeFw = INIT_VAL_ZERO;
	gl_counterdot = INIT_VAL_ZERO;
	uint16_t  TOTAL_PACKETS = INIT_VAL_ZERO;
	// Init OLED display
	SSD1306_Init();
	/*Run main Appication before go to Flashing Appl*/
	// Init Flashing
	UI_Init_Flashing_Screen();
	RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);


}

void UI_Main_FLASHING(void){
	switch (gl_UISTATE) {
		/* UI_IDE STATE */
		case UI_IDLE:
		{
			//RTE_RUNNABLE_FLAG_NODE_REQUEST_INDEX_ReadData(&gl_u8NodeIndex);
			RTE_RUNNABLE_FLAG_NODE_STATUS_ReadData_NODE_1(&gl_u8NodeStatus_1);
			RTE_RUNNABLE_FLAG_NODE_STATUS_ReadData_NODE_2(&gl_u8NodeStatus_2);
			RTE_RUNNABLE_FLAG_NODE_STATUS_ReadData_NODE_3(&gl_u8NodeStatus_3);
			UI_Node_Connect(0x0,gl_u8NodeStatus_1);
			UI_Node_Connect(0x01,gl_u8NodeStatus_2);
			UI_Node_Connect(0x02,gl_u8NodeStatus_3);
			// Get System state Via RTE
			Std_ReturnType retVal  = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			if(RTE_E_OKE == retVal){
				if(SYS_RECEIVE_UPDATE == gl_SystemState )
				{
					// Clean UI screen
					UI_Clean();
					UI_WaitForResp();
					// Display Information for FLashing
					UI_Downloading_FW();
					gl_UISTATE =  UI_DOWNLOADING_FW;

				}
				else if(WAIT_FOR_ESP_CONNECT == gl_SystemState){
					UI_Clean();
					gl_UISTATE = UI_WAIT_FOR_WIFI;
				}
			}

			else{
				/*State In IDLE STATE and Change UI to ERROR STATE*/
				gl_UISTATE = UI_DISPLAYERROR;
			}
			break;
		}
		case UI_WAIT_FOR_WIFI:
		{
			Std_ReturnType retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			UI_Wait_ForConnect_WiFi();
			if(RTE_E_OKE == retVal){
				if(WIFI_CONNECTED == gl_SystemState )
				{
					UI_Clean();
					UI_WiFi_Connected();
					//Change to UI_IDE
					gl_UISTATE = UI_IDLE;
					HAL_Delay(2000);
					UI_Clean();
					UI_Init_Flashing_Screen();
				}
			}
			break;
		}
		case UI_DOWNLOADING_FW:
		{
			Std_ReturnType retVal = RTE_RUNNABLE_DOWNLOAD_PROGRESS_ReadData(&gl_u8DownLoadProgress);
			if( RTE_E_OKE == retVal){
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
				UI_UpdateDownloading_FW(gl_u8DownLoadProgress);
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
				if(gl_u8DownLoadProgress >100)
					gl_u8DownLoadProgress=100;
				if(100 == gl_u8DownLoadProgress)
				{
					//HAL_Delay(2000);
					UI_Clean();
					gl_UISTATE = UI_START_OTA;
				}
				retVal = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_UIERRORTYPE);
				if(gl_UIERRORTYPE ==UI_ERROR_RESET )
					gl_UISTATE = UI_DISPLAYERROR;
			}
			break;
		}
		case UI_START_OTA:
		{
			Std_ReturnType retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			retVal = Rte_PacketSendLoraNum_ReadData(&gl_TimeRequest);
			if (RTE_E_OKE == retVal){
				if(gl_SystemState == SYS_REQUEST_OTA){
					UI_StartOTA();
					UI_Wait_ACCEPT_OTA(gl_TimeRequest);
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
			if (gl_SystemState == SYS_CONFIG_LORA){
				gl_UISTATE = UI_SYNCONFIGURATION;
				UI_Clean();
				RTE_RUNNABLE_CONFIG_LORA_ReadData(&gl_u32configLoRa);
				u8SF = (gl_u32configLoRa >> SHIFT_16_BIT)& 0xFF ;
				u8BW = (gl_u32configLoRa >> SHIFT_8_BIT)& 0xFF ;
				u8CR = (gl_u32configLoRa >> SHIFT_0_BIT)& 0xFF ;

				uint8_t sf = SX1278_SpreadFactor_Values[u8SF];
				float bw = SX1278_Bandwidth_Values[u8BW];
				float cr = SX1278_CodingRate_Values[u8CR];
				data_rate = UI_CalculateLoRaDataRate(sf, bw, cr);
				transmission_time_per_packet = (PACKET_SIZE * 8.0) / data_rate;
				UI_DisplayConfig(u8BW ,u8SF ,u8CR);
			}
			else {
				/*OUT OF REQUEST*/
				gl_UISTATE=UI_DISPLAYERROR;
				UI_Clean();
			}
			break;
		}
		case UI_SYNCONFIGURATION:
		{	Std_ReturnType retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			retVal = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_u8ErrorFlag);
			if( RTE_E_OKE == retVal){
				if(gl_SystemState == SYS_CONFIG_LORA){
					UI_Synconfiguaraton();
				}
				else if(gl_SystemState == SYS_SEND_UPDATE){
					UI_Clean();
					RTE_RUNNABLE_FLAG_TOTAL_REQUEST_PACKET_ReadData(&TOTAL_PACKETS);
					gl_UISTATE = UI_SENDPACKET_LORA_FW;
				}

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
				Std_ReturnType retVal_2 = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_u8ErrorFlag);
				UI_SendSW_LoRa(gl_u16Packet_LoRa_FW);
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

			UI_DoneDownload_FW();
			//HAL_Delay(2000);
			//UI_Init_Flashing_Screen();
			//RTE_RUNNABLE_SYSTEM_STATE_WriteData(SYS_IDLE);
			uint8_t retVal = RTE_RUNNABLE_SYSTEM_STATE_ReadData(&gl_SystemState);
			if(gl_SystemState ==SYS_IDLE ){
				UI_Clean();
				UI_Init_Flashing_Screen();
				gl_UISTATE = UI_IDLE;
			}

			break;
		}
		case UI_DISPLAYERROR:
		{
			/*ERROR Handler */
			Std_ReturnType retVal = RTE_RUNNABLE_UI_ERROR_ReadData(&gl_u8ErrorFlag);
			if(RTE_E_OKE == retVal){
				UI_DisplayERROR(gl_u8ErrorFlag);
				// START TIMER FOR RS SW
				HAL_TIM_Base_Start_IT(&htim4);
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
static void UI_Init_Flashing_Screen(void) {
    /* Set Background color */
    SSD1306_GotoXY(5, 0);
    SSD1306_Puts("GATEWAY", &Font_11x18, 1); // Print start FUOTA
    SSD1306_GotoXY(10, 20);
    SSD1306_Puts("Node 1: Loss", &Font_7x10, 1); // Initial status for Node 1
    SSD1306_GotoXY(10, 30);
    SSD1306_Puts("Node 2: Loss", &Font_7x10, 1); // Initial status for Node 2
    SSD1306_GotoXY(10, 40);
    SSD1306_Puts("Node 3: Loss", &Font_7x10, 1); // Initial status for Node 3
    SSD1306_UpdateScreen(); // Update screen
}

static void UI_Node_Connect(int nodeIndex, bool connected) {
    SSD1306_GotoXY(10, 20 + (nodeIndex * 10)); // Calculate Y position based on node index
    if (connected) {
        SSD1306_Puts("Node ", &Font_7x10, 1);
        SSD1306_Putc(nodeIndex + '1', &Font_7x10, 1);
        SSD1306_Puts(": Alive ", &Font_7x10, 1);
    } else {
        SSD1306_Puts("Node ", &Font_7x10, 1);
        SSD1306_Putc(nodeIndex + '1', &Font_7x10, 1);
        SSD1306_Puts(": Loss  ", &Font_7x10, 1);
    }
    SSD1306_UpdateScreen(); // Update screen
}
static void UI_Clean(void){
	SSD1306_Clear();
	SSD1306_UpdateScreen();
}

static void UI_WaitForResp(void){
	uint32_t local_u32NodeAddr = 0U;
	uint32_t local_u32Codesize = 0U;
	uint16_t local_u16Appver=0U;
	RTE_RUNNABLE_CODE_SIZE_ReadData(&local_u32Codesize);
	RTE_RUNNABLE_APP_VER_ReadData(&local_u16Appver);
	RTE_RUNNABLE_NODE_ADDR_ReadData(&local_u32NodeAddr);
	char buffer_version[10];
	char buffer_data_id[10];

	uint8_t major_version = (local_u16Appver >> 8) & 0xFF; // Extract major version (e.g., 1 from 0x00000131)
	uint8_t minor_version = local_u16Appver & 0xFF; // Extract minor version (e.g., 3 from 0x00000131)
	sprintf(buffer_data_id, "%X", local_u32NodeAddr);
	sprintf(buffer_version, "%d.%d", major_version, minor_version);
	char Local_DataBuffer[4];
	uint8_t local_estimatime = local_u32Codesize/BandWidth_UART +20;
	SSD1306_GotoXY (5, 0);
	SSD1306_Puts ("UPDATE DOWNLOAD!!", &Font_7x10, 1);
	sprintf(Local_DataBuffer , "%d" ,local_u16Appver );
	SSD1306_GotoXY (5, 10);
	SSD1306_Puts ("VERSION:", &Font_7x10, 1);
	SSD1306_GotoXY (70, 10);
	SSD1306_Puts (buffer_version, &Font_7x10, 1);
	SSD1306_GotoXY (5, 20);
	SSD1306_Puts ("NODE ID:", &Font_7x10, 1);
	SSD1306_GotoXY (60, 20);
	SSD1306_Puts (buffer_data_id, &Font_7x10, 1);
	SSD1306_UpdateScreen(); // update screen
}
static void UI_Downloading_FW(void){
	/* Writeing Text */
	SSD1306_GotoXY (30, 30);
	SSD1306_Puts ("PROGRESS", &Font_7x10, 1);
}
static void UI_UpdateDownloading_FW(uint8_t Var_Progress){
	char local_UpdateBuffer[4];
	if(Var_Progress >100)
		Var_Progress =100;
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
	char buffer_version[10];
	char buffer_data_id[10];
	uint32_t local_u32NodeAddr = 0U;
	RTE_RUNNABLE_NODE_ADDR_ReadData(&local_u32NodeAddr);
	uint8_t major_version = (gl_u32Version >> 8) & 0xFF; // Extract major version (e.g., 1 from 0x00000131)
	uint8_t minor_version = gl_u32Version & 0xFF; // Extract minor version (e.g., 3 from 0x00000131)
	sprintf(buffer_version, "%d.%d", major_version, minor_version);
	sprintf(buffer_data_id, "%X", local_u32NodeAddr);
	//char local_UpdateBuffer[4];
	SSD1306_GotoXY (40, 10);
	SSD1306_Puts ("FUOTA", &Font_11x18, 1);
	SSD1306_GotoXY (30, 30);
	SSD1306_Puts ("COMPLETED", &Font_7x10, 1);
	SSD1306_GotoXY (30, 40);

	SSD1306_Puts ("VERSION: ", &Font_7x10, 1);
	SSD1306_GotoXY (100, 40);
	SSD1306_Puts (buffer_version, &Font_7x10, 1);
	SSD1306_GotoXY (5, 50);
	SSD1306_Puts ("NODE ID:", &Font_7x10, 1);
	SSD1306_GotoXY (60, 50);
	SSD1306_Puts (buffer_data_id, &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}

static void UI_Synconfiguaraton(void){
	static uint8_t dot_count = 0; // Biến tĩnh để theo dõi trạng thái của hiệu ứng
	char dots[4]; // Chuỗi chứa hiệu ứng chấm
	// Cập nhật chuỗi hiệu ứng chấm dựa trên dot_count
	 switch (dot_count) {
	 case 0:
		 strcpy(dots, ".");
		 dot_count++;
		 break;
	 case 1:
		 strcpy(dots, "..");
		 dot_count++;
		 break;
	 case 2:
		 strcpy(dots, "...");
		 dot_count = 0; // Quay lại trạng thái đầu tiên sau khi đạt tới "..."
		 break;
	 default:
		 strcpy(dots, ""); // Trường hợp mặc định (không bao giờ xảy ra)
		 break;
	 }
	 // Chỉ xóa dòng cần cập nhật thay vì toàn bộ màn hình
	 SSD1306_GotoXY(60, 0);
	 SSD1306_Puts("   ", &Font_7x10, 1); // Xóa các dấu chấm cũ
	 SSD1306_GotoXY(5, 10);
	 SSD1306_Puts("SYNC UP ", &Font_7x10, 1);
	 SSD1306_GotoXY(60, 10);
	 SSD1306_Puts(dots, &Font_7x10, 1);
	SSD1306_UpdateScreen(); // Cập nhật màn hình
}
static void UI_SendSW_LoRa(uint16_t Var_numPacket) {

    static uint8_t dot_count = 0; // Biến tĩnh để theo dõi trạng thái của hiệu ứng
    char dots[4]; // Chuỗi chứa hiệu ứng chấm
    char local_UpdateBuffer[4];
    // Cập nhật chuỗi hiệu ứng chấm dựa trên dot_count
    switch (dot_count) {
        case 0:
            strcpy(dots, ".");
            dot_count++;
            break;
        case 1:
            strcpy(dots, "..");
            dot_count++;
            break;
        case 2:
            strcpy(dots, "...");
            dot_count = 0; // Quay lại trạng thái đầu tiên sau khi đạt tới "..."
            break;
        default:
            strcpy(dots, ""); // Trường hợp mặc định (không bao giờ xảy ra)
            break;
    }
    // Calculate remain time
    float remaining_time = (transmission_time_per_packet+0.14) * (TOTAL_PACKETS - Var_numPacket);
    // Chuyển đổi thời gian còn lại sang phút và giây
    int minutes = (int)(remaining_time / 60);
    int seconds = (int)(remaining_time) % 60;
    char remaining_time_str[10];
    sprintf(remaining_time_str, "%02dm %02ds", minutes, seconds);
    // Chuẩn bị nội dung để hiển thị
    sprintf(local_UpdateBuffer, "%d", Var_numPacket);

    // Chỉ xóa phần cần cập nhật thay vì toàn bộ màn hình
    SSD1306_GotoXY(5, 0);
    SSD1306_Puts("UPDATING", &Font_7x10, 1);
    SSD1306_GotoXY(80, 0); // Vị trí bắt đầu của dấu chấm
    SSD1306_Puts("   ", &Font_7x10, 1); // Xóa dấu chấm cũ
    SSD1306_GotoXY(80, 0); // Vị trí bắt đầu của dấu chấm
    SSD1306_Puts(dots, &Font_7x10, 1);
    SSD1306_GotoXY(5, 20);
    SSD1306_Puts("TIME LEFT:", &Font_7x10, 1);
    SSD1306_GotoXY(80, 20);
    SSD1306_Puts(remaining_time_str, &Font_7x10, 1);
    SSD1306_GotoXY(5, 40);
    SSD1306_Puts("PACKET SEND:", &Font_7x10, 1);
    SSD1306_GotoXY(100, 40);
    SSD1306_Puts(local_UpdateBuffer, &Font_7x10, 1);
    // Tính toán phần trăm hoàn thành

    float progress = (float)Var_numPacket / TOTAL_PACKETS;
    int progress_bar_length = (int)(progress * PROGRESS_BAR_WIDTH);
    // Vẽ thanh tiến trình
    SSD1306_DrawRectangle(5, 50, PROGRESS_BAR_WIDTH, 10, SSD1306_COLOR_WHITE); // Vẽ khung
    SSD1306_DrawFilledRectangle(5, 50, progress_bar_length, 10, SSD1306_COLOR_WHITE); // Vẽ thanh tiến trình
    SSD1306_UpdateScreen(); // Cập nhật màn hình
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
	SSD1306_GotoXY (50, 10);
	SSD1306_Puts ("ERROR :", &Font_7x10, 1);
	SSD1306_GotoXY (10, 30);
	switch (Var_UIError) {
		case MCU_ERROR_CRC:
			SSD1306_Puts ("INVALID CRC", &Font_7x10, 1);
			break;
		case GW_REQUEST_UPDATE_ERROR:
			SSD1306_Puts ("REQUEST UPDATE", &Font_7x10, 1);
			SSD1306_GotoXY (50, 40);
			SSD1306_Puts ("ERROR", &Font_7x10, 1);
			break;
		case GW_OUTOFREQUEST_ERROR:
			SSD1306_Puts ("REQUEST UPDATE", &Font_7x10, 1);
			SSD1306_GotoXY (50, 40);
			SSD1306_Puts ("FAIL", &Font_7x10, 1);
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
	SSD1306_GotoXY (5, 10);
	SSD1306_Puts ("UPDATING...", &Font_7x10, 1);
	SSD1306_GotoXY (5, 20);
	SSD1306_Puts ("PACKET RESENT:", &Font_7x10, 1);
	SSD1306_GotoXY (100, 30);
	SSD1306_Puts (local_UpdateBuffer, &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}


static void UI_DisplayConfig(uint8_t bw, uint8_t sf, uint8_t cr) {
    char buffer[30]; // Buffer để chứa chuỗi hiển thị

    // Xóa màn hình trước khi cập nhật
    SSD1306_Clear();
    // Hiển thị tiêu đề
    SSD1306_GotoXY(5, 20);
    SSD1306_Puts("LoRa Config", &Font_7x10, 1);
    // Hiển thị giá trị Bandwidth (BW)
    sprintf(buffer, "BW: %s kHz", SX1278_Bandwidth_UI[bw]);
    SSD1306_GotoXY(5, 30);
    SSD1306_Puts(buffer, &Font_7x10, 1);
    // Hiển thị giá trị Spread Factor (SF)
    sprintf(buffer, "SF: %u", SX1278_SpreadFactor_UI[sf]);
    SSD1306_GotoXY(5, 40);
    SSD1306_Puts(buffer, &Font_7x10, 1);

    // Hiển thị giá trị Coding Rate (CR)
    sprintf(buffer, "CR: %s", SX1278_CodingRate_UI[cr]);
    SSD1306_GotoXY(5, 50);
    SSD1306_Puts(buffer, &Font_7x10, 1);

    // Cập nhật màn hình
    SSD1306_UpdateScreen();
}

static void UI_Wait_ForConnect_WiFi(){
	// Variable to track the dot effect
	static uint8_t dot_count = 0;
	char dots[4];
	switch (dot_count) {
	case 0:
		strcpy(dots, ".");
		dot_count++;
		break;
	case 1:
		strcpy(dots, "..");
		dot_count++;
		break;
	case 2:
		strcpy(dots, "...");
		dot_count = 0; // Reset after reaching "..."
		break;
	default:
		strcpy(dots, ""); // Default case (should not occur)
		break;
	}
	 // Clear only the necessary area of the screen instead of the whole screen
	SSD1306_GotoXY(90, 30);
	SSD1306_Puts("   ", &Font_7x10, 1); // Clear old dots
	// Display the static message
    /* Set Background color */
    SSD1306_GotoXY(5, 0);
    SSD1306_Puts("GATEWAY", &Font_11x18, 1); // Print start FUOTA
	SSD1306_GotoXY(5, 20);
	SSD1306_Puts("Wait for", &Font_7x10, 1);
	SSD1306_GotoXY(5, 30);
	SSD1306_Puts("Connect WiFi", &Font_7x10, 1);
	// Display the dots
	SSD1306_GotoXY(90, 30);
	SSD1306_Puts(dots, &Font_7x10, 1);
	// Update the screen
	SSD1306_UpdateScreen();
}
static void UI_WiFi_Connected(){
	// Variable to track the dot effect
	SSD1306_GotoXY(5, 0);
	SSD1306_Puts("GATEWAY", &Font_11x18, 1); // Print start FUOTA
	// Display the static message
	SSD1306_GotoXY(5, 20);
	SSD1306_Puts("WIFI ", &Font_7x10, 1);
	SSD1306_GotoXY(5, 30);
	SSD1306_Puts("CONNECTED !!", &Font_7x10, 1);
	SSD1306_UpdateScreen();
}
float UI_CalculateLoRaDataRate(uint8_t sf, uint32_t bw, uint8_t cr) {
	return sf * ((4.0 / (4.0 + cr)) / (pow(2.0, sf) / bw)) * 1000.0;
}

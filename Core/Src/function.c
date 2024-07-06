/*
 * function.c
 *
 *  Created on: Apr 18, 2024
 *      Author: quang
 */
#include "function.h"

void FUNC_Blink_Led_Receive(uint32_t TIMER){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
	//HAL_Delay(TIMER);
}

void FUNC_Blink_Led_Send(uint32_t TIMER){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	HAL_Delay(TIMER);
}

void FUNC_get_DHT_val(uint8_t * buffer){
	float Temp,Humi=0;
	if (DHT_Start()) {
	    uint8_t RHI  = DHT_Read();
	    uint8_t RHD  = DHT_Read();
	    uint8_t TCI  = DHT_Read();
	    uint8_t TCD  = DHT_Read();
	    uint8_t SUM  = DHT_Read();
	    if (RHI + RHD + TCI + TCD == SUM)
	    {
	    	// Can use RHI and TCI for any purposes if whole number only needed
	    	// DHT22
//	    	if (TCI>127) // If TC1=10000000, negative temperature
//	    	{
//	    		Temp = (float)TCD/10*(-1);
//	    	}
//	    	else
//	    	{
//	    		Temp = (float)((TCI<<8)|TCD)/10;
//	    	        }
//	    	Humi = (float) ((RHI<<8)|RHD)/10;
	    	float Temp = (float)TCI + (float)(TCD/10.0);
	    	float Humi = (float)RHI  + (float)(RHD/10.0);
	    	// Can use tCelsius, tFahrenheit and RH for any purposes
	   	 //DHT11 FORMAT
	   		 uint16_t temp_int =(uint16_t)(Temp);
	   		 uint16_t temp_frac = (int16_t)((Temp - temp_int) * 100);
	   		 uint16_t humi_int =(uint16_t)(Humi);
	   		 uint16_t humi_frac = (int16_t)((Humi - humi_int) * 100);
	   		 buffer[0] = (uint8_t)(temp_int); // Lower byte of temperature
	   		 buffer[1] = (uint8_t)(temp_frac); // Higher byte of temperature
	   		 buffer[2] = (uint8_t)(humi_int); // Lower byte of humidity
	   		 buffer[3] = (uint8_t)(humi_frac); // Higher byte of humidity
	   		 //DHT22
//                                                                                   	   		 uint8_t temp_integral = (uint8_t)(Temp); // 32
//	   		 uint8_t temp_fractional = (uint8_t)((int)Temp % 10); // 5
//	   		 uint8_t humi_integral = (uint8_t)(Humi); // 69
//	   		 uint8_t humi_fractional = (uint8_t)((int)Humi % 10); // 2
//	   		 buffer[0] = temp_integral; // 32
//	   		 buffer[1] = temp_fractional; // 5
//	   		 buffer[2] = humi_integral; // 69
//	   		 buffer[3] = humi_fractional; // 2
	    }


	}
	 HAL_Delay(2000);

}

//uint16_t FUNC_get_MQ_val(){
//	 uint16_t MQ2_Val;
//	 ADC_Select_CH1();
//	 HAL_ADC_Start(&hadc1);
//	 HAL_ADC_PollForConversion(&hadc1, 1000);
//	 MQ2_Val = HAL_ADC_GetValue(&hadc1);
//	 HAL_ADC_Stop(&hadc1);
//	 return MQ2_Val;
//}
uint32_t FUNC_ReaddataAddress(uint32_t Address){
	uint32_t Local_u32AddressData = *((volatile uint32_t*)(Address));
	return Local_u32AddressData;
}

void FUNC_EraseAndRestore_Header_Page(uint32_t Copy_u32Address, uint32_t Copy_u32NewData){

	uint32_t Local_u32AddressArray	[NUMBER_OF_FLAGS];
	uint32_t Local_u32DataArray		[NUMBER_OF_FLAGS];
	uint16_t Local_u16DataIndex        = 0;
	uint16_t Local_u16DataCounter      = 0;
	uint32_t Local_u32AddressCounter   = 0;

	//Copy all flag to array before erase
	for(Local_u32AddressCounter = START_OF_FLAG_REGION ;Local_u32AddressCounter < END_OF_FLAG_REGION;)
	{
		if((Local_u32AddressCounter != Copy_u32Address) & (*((volatile uint32_t*)(Local_u32AddressCounter)) != ERASED_VALUE))
		{
			Local_u32AddressArray[Local_u16DataIndex] = Local_u32AddressCounter;
			Local_u32DataArray[Local_u16DataIndex] = *((volatile uint32_t*)(Local_u32AddressCounter));
			Local_u16DataIndex++ ;
		}
		Local_u32AddressCounter = Local_u32AddressCounter + WORD_SIZE_IN_BYTE;
	}

	// Erase the Flag region.
	FLASH_EraseInitTypeDef Local_eraseInfo;
	uint32_t Local_u32PageError;
	Local_eraseInfo.TypeErase = FLASH_TYPEERASE_PAGES;
	Local_eraseInfo.Banks = FLASH_BANK_1;
	Local_eraseInfo.PageAddress = FLAG_IMAGE;
	Local_eraseInfo.NbPages = 1;
	HAL_FLASH_Unlock(); //Unlocks the flash memory
	HAL_FLASHEx_Erase(&Local_eraseInfo, &Local_u32PageError); //Deletes given sectors
	for (Local_u16DataCounter = 0 ; Local_u16DataCounter < Local_u16DataIndex ; Local_u16DataCounter++ )
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Local_u32AddressArray[Local_u16DataCounter], Local_u32DataArray[Local_u16DataCounter]);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Copy_u32Address, Copy_u32NewData); //Replace new data to flash
	HAL_FLASH_Lock();  //Locks again the flash memory
}

void FUNC_voidMakeSoftWareReset(void)
{
	// make software reset after flashing success
#ifdef Debug
	__HAL_DBGMCU_FREEZE_IWDG();
#endif
	NVIC_SystemReset();
}



/************** MAKE CHANGES HERE ********************/
#include "stm32f1xx_hal.h"
#include "DHT.h"

#define DHT11 11
#define DHT22 22

#define DHT_PORT GPIOB
#define DHT_PIN GPIO_PIN_5

uint32_t pMillis, cMillis;

// Định nghĩa để chọn loại cảm biến
#define DHT_SENSOR DHT11 // Thay đổi thành DHT11 nếu muốn sử dụng DHT11

#if (DHT_SENSOR == DHT22)
#define DHT_PORT GPIOB
#define DHT_PIN GPIO_PIN_5
#elif (DHT_SENSOR == DHT11)
#define DHT_PORT GPIOB
#define DHT_PIN GPIO_PIN_5
#endif

uint32_t pMillis, cMillis;

void microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT_Start(void) {
    uint8_t Response = 0;
    GPIO_InitTypeDef GPIO_InitStructPrivate = {0};

    GPIO_InitStructPrivate.Pin = DHT_PIN;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStructPrivate); // set the pin as output

    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, 0); // pull the pin low

    #if (DHT_SENSOR == DHT22)
    microDelay(1300); // wait for 1300us for DHT22
    #elif (DHT_SENSOR == DHT11)
    HAL_Delay(20); // wait for 20ms for DHT11
    #endif

    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, 1); // pull the pin high
    microDelay(30); // wait for 30us

    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStructPrivate); // set the pin as input

    microDelay(40);

    if (!(HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN))) {
        microDelay(80);
        if ((HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN))) Response = 1;
    }

    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)) && pMillis + 2 > cMillis) {
        cMillis = HAL_GetTick();
    }

    return Response;
}

uint8_t DHT_Read(void) {
    uint8_t a, b = 0;
    for (a = 0; a < 8; a++) {
        pMillis = HAL_GetTick();
        cMillis = HAL_GetTick();
        while (!(HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)) && pMillis + 2 > cMillis) {
            cMillis = HAL_GetTick();
        }
        microDelay(40); // wait for 40 us
        if (!(HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN))) // if the pin is low
            b &= ~(1 << (7 - a));
        else
            b |= (1 << (7 - a));

        pMillis = HAL_GetTick();
        cMillis = HAL_GetTick();
        while ((HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)) && pMillis + 2 > cMillis) {
            cMillis = HAL_GetTick();
        }
    }
    return b;
}

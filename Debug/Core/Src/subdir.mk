################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DHT.c \
../Core/Src/SSD1306.c \
../Core/Src/SX1278.c \
../Core/Src/SX1278_hw.c \
../Core/Src/SX1278_if.c \
../Core/Src/UserInterface.c \
../Core/Src/aes.c \
../Core/Src/delay_timer.c \
../Core/Src/fonts.c \
../Core/Src/function.c \
../Core/Src/main.c \
../Core/Src/operation.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/DHT.o \
./Core/Src/SSD1306.o \
./Core/Src/SX1278.o \
./Core/Src/SX1278_hw.o \
./Core/Src/SX1278_if.o \
./Core/Src/UserInterface.o \
./Core/Src/aes.o \
./Core/Src/delay_timer.o \
./Core/Src/fonts.o \
./Core/Src/function.o \
./Core/Src/main.o \
./Core/Src/operation.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/DHT.d \
./Core/Src/SSD1306.d \
./Core/Src/SX1278.d \
./Core/Src/SX1278_hw.d \
./Core/Src/SX1278_if.d \
./Core/Src/UserInterface.d \
./Core/Src/aes.d \
./Core/Src/delay_timer.d \
./Core/Src/fonts.d \
./Core/Src/function.d \
./Core/Src/main.d \
./Core/Src/operation.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DHT.cyclo ./Core/Src/DHT.d ./Core/Src/DHT.o ./Core/Src/DHT.su ./Core/Src/SSD1306.cyclo ./Core/Src/SSD1306.d ./Core/Src/SSD1306.o ./Core/Src/SSD1306.su ./Core/Src/SX1278.cyclo ./Core/Src/SX1278.d ./Core/Src/SX1278.o ./Core/Src/SX1278.su ./Core/Src/SX1278_hw.cyclo ./Core/Src/SX1278_hw.d ./Core/Src/SX1278_hw.o ./Core/Src/SX1278_hw.su ./Core/Src/SX1278_if.cyclo ./Core/Src/SX1278_if.d ./Core/Src/SX1278_if.o ./Core/Src/SX1278_if.su ./Core/Src/UserInterface.cyclo ./Core/Src/UserInterface.d ./Core/Src/UserInterface.o ./Core/Src/UserInterface.su ./Core/Src/aes.cyclo ./Core/Src/aes.d ./Core/Src/aes.o ./Core/Src/aes.su ./Core/Src/delay_timer.cyclo ./Core/Src/delay_timer.d ./Core/Src/delay_timer.o ./Core/Src/delay_timer.su ./Core/Src/fonts.cyclo ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/function.cyclo ./Core/Src/function.d ./Core/Src/function.o ./Core/Src/function.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/operation.cyclo ./Core/Src/operation.d ./Core/Src/operation.o ./Core/Src/operation.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src


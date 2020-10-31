################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/crc.c \
../Core/Inc/modbus.c \
../Core/Inc/rs485.c 

OBJS += \
./Core/Inc/crc.o \
./Core/Inc/modbus.o \
./Core/Inc/rs485.o 

C_DEPS += \
./Core/Inc/crc.d \
./Core/Inc/modbus.d \
./Core/Inc/rs485.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o: ../Core/Inc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F411xE -I"C:/GoogleDrive/Elektronika/AC fan driver/stm32_app/CubeMx project/Core/Inc" -I"C:/GoogleDrive/Elektronika/AC fan driver/stm32_app/CubeMx project/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/GoogleDrive/Elektronika/AC fan driver/stm32_app/CubeMx project/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/GoogleDrive/Elektronika/AC fan driver/stm32_app/CubeMx project/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/GoogleDrive/Elektronika/AC fan driver/stm32_app/CubeMx project/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



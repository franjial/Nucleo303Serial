################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/periph_uart.c 

OBJS += \
./src/periph_uart.o 

C_DEPS += \
./src/periph_uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F3 -DSTM32F30 -DSTM32F303K8Tx -DNUCLEO_F303K8 -DDEBUG -DSTM32F303x8 -DUSE_HAL_DRIVER -I"C:/Users/francisco/workspace_stm32/Nucleo303Serial/inc" -I"C:/Users/francisco/workspace_stm32/nucleo-f303k8_hal_lib" -I"C:/Users/francisco/workspace_stm32/nucleo-f303k8_hal_lib/CMSIS/core" -I"C:/Users/francisco/workspace_stm32/nucleo-f303k8_hal_lib/CMSIS/device" -I"C:/Users/francisco/workspace_stm32/nucleo-f303k8_hal_lib/HAL_Driver/Inc/Legacy" -I"C:/Users/francisco/workspace_stm32/nucleo-f303k8_hal_lib/HAL_Driver/Inc" -I"C:/Users/francisco/workspace_stm32/nucleo-f303k8_hal_lib/Utilities/STM32F3xx_Nucleo_32" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



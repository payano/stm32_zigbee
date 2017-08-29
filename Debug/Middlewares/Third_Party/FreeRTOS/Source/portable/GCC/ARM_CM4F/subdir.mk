################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F303xC -I"/home/johan/workspace_stm32/yolo/Inc" -I"/home/johan/workspace_stm32/yolo/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/johan/workspace_stm32/yolo/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/home/johan/workspace_stm32/yolo/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/johan/workspace_stm32/yolo/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/johan/workspace_stm32/yolo/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/johan/workspace_stm32/yolo/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/johan/workspace_stm32/yolo/Drivers/CMSIS/Include" -I"/home/johan/workspace_stm32/yolo/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



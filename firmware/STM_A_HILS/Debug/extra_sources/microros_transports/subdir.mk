################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../micro_ros_stm32cubemx_utils/extra_sources/microros_transports/uart_transport.c 

OBJS += \
./extra_sources/microros_transports/uart_transport.o 

C_DEPS += \
./extra_sources/microros_transports/uart_transport.d 


# Each subdirectory must supply rules for building sources it contributes
extra_sources/microros_transports/uart_transport.o: /home/seo/KAI_26/workspace/Control/firmware/STM_A_HILS/micro_ros_stm32cubemx_utils/extra_sources/microros_transports/uart_transport.c extra_sources/microros_transports/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-extra_sources-2f-microros_transports

clean-extra_sources-2f-microros_transports:
	-$(RM) ./extra_sources/microros_transports/uart_transport.cyclo ./extra_sources/microros_transports/uart_transport.d ./extra_sources/microros_transports/uart_transport.o ./extra_sources/microros_transports/uart_transport.su

.PHONY: clean-extra_sources-2f-microros_transports


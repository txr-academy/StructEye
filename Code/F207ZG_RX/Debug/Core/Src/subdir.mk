################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MQTTClient.c \
../Core/Src/MQTTConnectClient.c \
../Core/Src/MQTTConnectServer.c \
../Core/Src/MQTTDeserializePublish.c \
../Core/Src/MQTTFormat.c \
../Core/Src/MQTTPacket.c \
../Core/Src/MQTTSerializePublish.c \
../Core/Src/MQTTSubscribeClient.c \
../Core/Src/MQTTSubscribeServer.c \
../Core/Src/MQTTUnsubscribeClient.c \
../Core/Src/MQTTUnsubscribeServer.c \
../Core/Src/freertos.c \
../Core/Src/lora.c \
../Core/Src/main.c \
../Core/Src/mqtt_transport.c \
../Core/Src/stm32f2xx_hal_msp.c \
../Core/Src/stm32f2xx_hal_timebase_tim.c \
../Core/Src/stm32f2xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f2xx.c \
../Core/Src/tcp_server.c 

OBJS += \
./Core/Src/MQTTClient.o \
./Core/Src/MQTTConnectClient.o \
./Core/Src/MQTTConnectServer.o \
./Core/Src/MQTTDeserializePublish.o \
./Core/Src/MQTTFormat.o \
./Core/Src/MQTTPacket.o \
./Core/Src/MQTTSerializePublish.o \
./Core/Src/MQTTSubscribeClient.o \
./Core/Src/MQTTSubscribeServer.o \
./Core/Src/MQTTUnsubscribeClient.o \
./Core/Src/MQTTUnsubscribeServer.o \
./Core/Src/freertos.o \
./Core/Src/lora.o \
./Core/Src/main.o \
./Core/Src/mqtt_transport.o \
./Core/Src/stm32f2xx_hal_msp.o \
./Core/Src/stm32f2xx_hal_timebase_tim.o \
./Core/Src/stm32f2xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f2xx.o \
./Core/Src/tcp_server.o 

C_DEPS += \
./Core/Src/MQTTClient.d \
./Core/Src/MQTTConnectClient.d \
./Core/Src/MQTTConnectServer.d \
./Core/Src/MQTTDeserializePublish.d \
./Core/Src/MQTTFormat.d \
./Core/Src/MQTTPacket.d \
./Core/Src/MQTTSerializePublish.d \
./Core/Src/MQTTSubscribeClient.d \
./Core/Src/MQTTSubscribeServer.d \
./Core/Src/MQTTUnsubscribeClient.d \
./Core/Src/MQTTUnsubscribeServer.d \
./Core/Src/freertos.d \
./Core/Src/lora.d \
./Core/Src/main.d \
./Core/Src/mqtt_transport.d \
./Core/Src/stm32f2xx_hal_msp.d \
./Core/Src/stm32f2xx_hal_timebase_tim.d \
./Core/Src/stm32f2xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f2xx.d \
./Core/Src/tcp_server.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/posix -I../Middlewares/Third_Party/LwIP/src/include/posix/sys -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MQTTClient.cyclo ./Core/Src/MQTTClient.d ./Core/Src/MQTTClient.o ./Core/Src/MQTTClient.su ./Core/Src/MQTTConnectClient.cyclo ./Core/Src/MQTTConnectClient.d ./Core/Src/MQTTConnectClient.o ./Core/Src/MQTTConnectClient.su ./Core/Src/MQTTConnectServer.cyclo ./Core/Src/MQTTConnectServer.d ./Core/Src/MQTTConnectServer.o ./Core/Src/MQTTConnectServer.su ./Core/Src/MQTTDeserializePublish.cyclo ./Core/Src/MQTTDeserializePublish.d ./Core/Src/MQTTDeserializePublish.o ./Core/Src/MQTTDeserializePublish.su ./Core/Src/MQTTFormat.cyclo ./Core/Src/MQTTFormat.d ./Core/Src/MQTTFormat.o ./Core/Src/MQTTFormat.su ./Core/Src/MQTTPacket.cyclo ./Core/Src/MQTTPacket.d ./Core/Src/MQTTPacket.o ./Core/Src/MQTTPacket.su ./Core/Src/MQTTSerializePublish.cyclo ./Core/Src/MQTTSerializePublish.d ./Core/Src/MQTTSerializePublish.o ./Core/Src/MQTTSerializePublish.su ./Core/Src/MQTTSubscribeClient.cyclo ./Core/Src/MQTTSubscribeClient.d ./Core/Src/MQTTSubscribeClient.o ./Core/Src/MQTTSubscribeClient.su ./Core/Src/MQTTSubscribeServer.cyclo ./Core/Src/MQTTSubscribeServer.d ./Core/Src/MQTTSubscribeServer.o ./Core/Src/MQTTSubscribeServer.su ./Core/Src/MQTTUnsubscribeClient.cyclo ./Core/Src/MQTTUnsubscribeClient.d ./Core/Src/MQTTUnsubscribeClient.o ./Core/Src/MQTTUnsubscribeClient.su ./Core/Src/MQTTUnsubscribeServer.cyclo ./Core/Src/MQTTUnsubscribeServer.d ./Core/Src/MQTTUnsubscribeServer.o ./Core/Src/MQTTUnsubscribeServer.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/lora.cyclo ./Core/Src/lora.d ./Core/Src/lora.o ./Core/Src/lora.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mqtt_transport.cyclo ./Core/Src/mqtt_transport.d ./Core/Src/mqtt_transport.o ./Core/Src/mqtt_transport.su ./Core/Src/stm32f2xx_hal_msp.cyclo ./Core/Src/stm32f2xx_hal_msp.d ./Core/Src/stm32f2xx_hal_msp.o ./Core/Src/stm32f2xx_hal_msp.su ./Core/Src/stm32f2xx_hal_timebase_tim.cyclo ./Core/Src/stm32f2xx_hal_timebase_tim.d ./Core/Src/stm32f2xx_hal_timebase_tim.o ./Core/Src/stm32f2xx_hal_timebase_tim.su ./Core/Src/stm32f2xx_it.cyclo ./Core/Src/stm32f2xx_it.d ./Core/Src/stm32f2xx_it.o ./Core/Src/stm32f2xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f2xx.cyclo ./Core/Src/system_stm32f2xx.d ./Core/Src/system_stm32f2xx.o ./Core/Src/system_stm32f2xx.su ./Core/Src/tcp_server.cyclo ./Core/Src/tcp_server.d ./Core/Src/tcp_server.o ./Core/Src/tcp_server.su

.PHONY: clean-Core-2f-Src


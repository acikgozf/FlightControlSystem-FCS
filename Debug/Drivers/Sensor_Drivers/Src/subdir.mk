################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Sensor_Drivers/Src/BMP180.c \
../Drivers/Sensor_Drivers/Src/GPS.c \
../Drivers/Sensor_Drivers/Src/RF.c \
../Drivers/Sensor_Drivers/Src/mpu6050.c \
../Drivers/Sensor_Drivers/Src/neo_m8n_gps.c \
../Drivers/Sensor_Drivers/Src/nrf24.c \
../Drivers/Sensor_Drivers/Src/quaternion_calculator.c \
../Drivers/Sensor_Drivers/Src/sensordriver.c 

OBJS += \
./Drivers/Sensor_Drivers/Src/BMP180.o \
./Drivers/Sensor_Drivers/Src/GPS.o \
./Drivers/Sensor_Drivers/Src/RF.o \
./Drivers/Sensor_Drivers/Src/mpu6050.o \
./Drivers/Sensor_Drivers/Src/neo_m8n_gps.o \
./Drivers/Sensor_Drivers/Src/nrf24.o \
./Drivers/Sensor_Drivers/Src/quaternion_calculator.o \
./Drivers/Sensor_Drivers/Src/sensordriver.o 

C_DEPS += \
./Drivers/Sensor_Drivers/Src/BMP180.d \
./Drivers/Sensor_Drivers/Src/GPS.d \
./Drivers/Sensor_Drivers/Src/RF.d \
./Drivers/Sensor_Drivers/Src/mpu6050.d \
./Drivers/Sensor_Drivers/Src/neo_m8n_gps.d \
./Drivers/Sensor_Drivers/Src/nrf24.d \
./Drivers/Sensor_Drivers/Src/quaternion_calculator.d \
./Drivers/Sensor_Drivers/Src/sensordriver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Sensor_Drivers/Src/%.o: ../Drivers/Sensor_Drivers/Src/%.c Drivers/Sensor_Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I"C:/Users/FURKAN/Desktop/STM32F429/FCS_v1.0/Drivers/Filters_and_Controllers/Inc" -I../Core/Inc -I"C:/Users/FURKAN/Desktop/STM32F429/FCS_v1.0/Drivers/Sensor_Drivers/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Sensor_Drivers-2f-Src

clean-Drivers-2f-Sensor_Drivers-2f-Src:
	-$(RM) ./Drivers/Sensor_Drivers/Src/BMP180.d ./Drivers/Sensor_Drivers/Src/BMP180.o ./Drivers/Sensor_Drivers/Src/GPS.d ./Drivers/Sensor_Drivers/Src/GPS.o ./Drivers/Sensor_Drivers/Src/RF.d ./Drivers/Sensor_Drivers/Src/RF.o ./Drivers/Sensor_Drivers/Src/mpu6050.d ./Drivers/Sensor_Drivers/Src/mpu6050.o ./Drivers/Sensor_Drivers/Src/neo_m8n_gps.d ./Drivers/Sensor_Drivers/Src/neo_m8n_gps.o ./Drivers/Sensor_Drivers/Src/nrf24.d ./Drivers/Sensor_Drivers/Src/nrf24.o ./Drivers/Sensor_Drivers/Src/quaternion_calculator.d ./Drivers/Sensor_Drivers/Src/quaternion_calculator.o ./Drivers/Sensor_Drivers/Src/sensordriver.d ./Drivers/Sensor_Drivers/Src/sensordriver.o

.PHONY: clean-Drivers-2f-Sensor_Drivers-2f-Src


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Filters_and_Controllers/Src/bldc_motor_driving.c \
../Drivers/Filters_and_Controllers/Src/lpf.c \
../Drivers/Filters_and_Controllers/Src/pid.c \
../Drivers/Filters_and_Controllers/Src/servo_control.c \
../Drivers/Filters_and_Controllers/Src/transfer_function.c 

OBJS += \
./Drivers/Filters_and_Controllers/Src/bldc_motor_driving.o \
./Drivers/Filters_and_Controllers/Src/lpf.o \
./Drivers/Filters_and_Controllers/Src/pid.o \
./Drivers/Filters_and_Controllers/Src/servo_control.o \
./Drivers/Filters_and_Controllers/Src/transfer_function.o 

C_DEPS += \
./Drivers/Filters_and_Controllers/Src/bldc_motor_driving.d \
./Drivers/Filters_and_Controllers/Src/lpf.d \
./Drivers/Filters_and_Controllers/Src/pid.d \
./Drivers/Filters_and_Controllers/Src/servo_control.d \
./Drivers/Filters_and_Controllers/Src/transfer_function.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Filters_and_Controllers/Src/%.o: ../Drivers/Filters_and_Controllers/Src/%.c Drivers/Filters_and_Controllers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I"C:/Users/FURKAN/Desktop/STM32F429/FCS_v1.0/Drivers/Filters_and_Controllers/Inc" -I../Core/Inc -I"C:/Users/FURKAN/Desktop/STM32F429/FCS_v1.0/Drivers/Sensor_Drivers/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Filters_and_Controllers-2f-Src

clean-Drivers-2f-Filters_and_Controllers-2f-Src:
	-$(RM) ./Drivers/Filters_and_Controllers/Src/bldc_motor_driving.d ./Drivers/Filters_and_Controllers/Src/bldc_motor_driving.o ./Drivers/Filters_and_Controllers/Src/lpf.d ./Drivers/Filters_and_Controllers/Src/lpf.o ./Drivers/Filters_and_Controllers/Src/pid.d ./Drivers/Filters_and_Controllers/Src/pid.o ./Drivers/Filters_and_Controllers/Src/servo_control.d ./Drivers/Filters_and_Controllers/Src/servo_control.o ./Drivers/Filters_and_Controllers/Src/transfer_function.d ./Drivers/Filters_and_Controllers/Src/transfer_function.o

.PHONY: clean-Drivers-2f-Filters_and_Controllers-2f-Src


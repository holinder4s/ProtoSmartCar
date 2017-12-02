################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/SHT15.c \
../Libraries/Touch.c \
../Libraries/lcd.c 

OBJS += \
./Libraries/SHT15.o \
./Libraries/Touch.o \
./Libraries/lcd.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/%.o: ../Libraries/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler 5'
	armcc -I"D:\ProtoSmartCar\ProtoSmartCar\Libraries" -I"D:\ProtoSmartCar\ProtoSmartCar\Libraries\STM32F10x_StdPeriph_Driver_v3.5\inc" -I"D:\ProtoSmartCar\ProtoSmartCar\Libraries\STM32F10x_StdPeriph_Driver_v3.5\src" -I"D:\ProtoSmartCar\ProtoSmartCar\Libraries\CMSIS\CoreSupport" -I"D:\ProtoSmartCar\ProtoSmartCar\Libraries\CMSIS\DeviceSupport" -I"D:\ProtoSmartCar\ProtoSmartCar\Libraries\CMSIS\DeviceSupport\Startup" -O1 --cpu=cortex-m3 -g -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



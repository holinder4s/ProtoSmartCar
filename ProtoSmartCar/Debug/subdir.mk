################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../proto_smartcar.c 

OBJS += \
./proto_smartcar.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler 5'
	armcc -I"C:\Users\hw_8\Documents\DS-5 Workspace\ProtoSmartCar\ProtoSmartCar\Libraries" -I"C:\Users\hw_8\Documents\DS-5 Workspace\ProtoSmartCar\ProtoSmartCar\Libraries\CMSIS\CoreSupport" -I"C:\Users\hw_8\Documents\DS-5 Workspace\ProtoSmartCar\ProtoSmartCar\Libraries\CMSIS\DeviceSupport" -I"C:\Users\hw_8\Documents\DS-5 Workspace\ProtoSmartCar\ProtoSmartCar\Libraries\CMSIS\DeviceSupport\Startup" -I"C:\Users\hw_8\Documents\DS-5 Workspace\ProtoSmartCar\ProtoSmartCar\Libraries\STM32F10x_StdPeriph_Driver_v3.5\inc" -I"C:\Users\hw_8\Documents\DS-5 Workspace\ProtoSmartCar\ProtoSmartCar\Libraries\STM32F10x_StdPeriph_Driver_v3.5\src" -O1 --cpu=cortex-m3 -g -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


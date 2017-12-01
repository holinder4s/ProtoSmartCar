################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := clean

# All of the sources participating in the build are defined here
-include sources.mk
-include Libraries/STM32F10x_StdPeriph_Driver_v3.5/src/subdir.mk
-include Libraries/CMSIS/DeviceSupport/Startup/subdir.mk
-include Libraries/CMSIS/DeviceSupport/subdir.mk
-include Libraries/CMSIS/CoreSupport/subdir.mk
-include Libraries/subdir.mk
-include subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: ProtoSmartCar.axf

# Tool invocations
ProtoSmartCar.axf: $(OBJS) C:\Users\hw_8\Documents\DS-5\ Workspace\ProtoSmartCar\ProtoSmartCar\stm32-test.sct $(USER_OBJS) $(LIBS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker 5'
	armlink --entry=main --cpu=cortex-m3 --scatter="C:\Users\hw_8\Documents\DS-5 Workspace\ProtoSmartCar\ProtoSmartCar\stm32-test.sct" --info=sizes -o "ProtoSmartCar.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) $(EXECUTABLES) ProtoSmartCar.axf 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

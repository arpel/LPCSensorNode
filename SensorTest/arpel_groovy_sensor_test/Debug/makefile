################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include example/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: arpel_groovy_sensor_test.axf

# Tool invocations
arpel_groovy_sensor_test.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"D:\Users\Arpel\git\LPCSensorNode\BOARDLib\arpel_groovy_sensor_board_lib\Debug" -L"D:\Users\Arpel\Documents\LPCXpresso_6.1.4_194\workspace\lpc_chip_8xx_lib\Debug" -Xlinker -Map="arpel_groovy_sensor_test.map" -Xlinker --gc-sections -mcpu=cortex-m0 -mthumb -T "arpel_groovy_sensor_test_Debug.ld" -o "arpel_groovy_sensor_test.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) arpel_groovy_sensor_test.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "arpel_groovy_sensor_test.axf"; # arm-none-eabi-objcopy -O binary "arpel_groovy_sensor_test.axf" "arpel_groovy_sensor_test.bin" ; checksum -p LPC812 -d "arpel_groovy_sensor_test.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets

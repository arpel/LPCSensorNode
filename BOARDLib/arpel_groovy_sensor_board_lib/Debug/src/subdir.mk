################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/board.c \
../src/board_sysinit.c \
../src/crc8.c \
../src/onewire.c 

OBJS += \
./src/board.o \
./src/board_sysinit.o \
./src/crc8.o \
./src/onewire.o 

C_DEPS += \
./src/board.d \
./src/board_sysinit.d \
./src/crc8.d \
./src/onewire.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -DCORE_M0PLUS -I"D:\Users\Arpel\Documents\LPCXpresso_6.1.4_194\workspace\lpc_chip_8xx_lib\inc" -I"D:\Users\Arpel\git\LPCSensorNode\BOARDLib\arpel_groovy_sensor_board_lib\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


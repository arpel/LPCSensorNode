################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/RF12.c \
../src/board.c \
../src/board_sysinit.c \
../src/circular_buffer.c \
../src/crc8.c \
../src/lpc8xx_sd_adc.c \
../src/onewire.c \
../src/simple_iir_lpf.c 

OBJS += \
./src/RF12.o \
./src/board.o \
./src/board_sysinit.o \
./src/circular_buffer.o \
./src/crc8.o \
./src/lpc8xx_sd_adc.o \
./src/onewire.o \
./src/simple_iir_lpf.o 

C_DEPS += \
./src/RF12.d \
./src/board.d \
./src/board_sysinit.d \
./src/circular_buffer.d \
./src/crc8.d \
./src/lpc8xx_sd_adc.d \
./src/onewire.d \
./src/simple_iir_lpf.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -DCORE_M0PLUS -I"D:\Users\Arpel\Documents\LPCXpresso_6.1.4_194\workspace\lpc_chip_8xx_lib\inc" -I"D:\Users\Arpel\git\LPCSensorNode\BOARDLib\arpel_groovy_sensor_board_lib\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



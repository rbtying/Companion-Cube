################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../libraries/Ethernet/utility/socket.cc \
../libraries/Ethernet/utility/w5100.cc 

OBJS += \
./libraries/Ethernet/utility/socket.o \
./libraries/Ethernet/utility/w5100.o 

CC_DEPS += \
./libraries/Ethernet/utility/socket.d \
./libraries/Ethernet/utility/w5100.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/Ethernet/utility/%.o: ../libraries/Ethernet/utility/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"}" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



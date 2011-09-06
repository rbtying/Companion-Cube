################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../libraries/Ethernet/Client.cc \
../libraries/Ethernet/Ethernet.cc \
../libraries/Ethernet/Server.cc \
../libraries/Ethernet/Udp.cc 

OBJS += \
./libraries/Ethernet/Client.o \
./libraries/Ethernet/Ethernet.o \
./libraries/Ethernet/Server.o \
./libraries/Ethernet/Udp.o 

CC_DEPS += \
./libraries/Ethernet/Client.d \
./libraries/Ethernet/Ethernet.d \
./libraries/Ethernet/Server.d \
./libraries/Ethernet/Udp.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/Ethernet/%.o: ../libraries/Ethernet/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"}" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



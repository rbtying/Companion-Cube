################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../libraries/Servo/Servo.cc 

OBJS += \
./libraries/Servo/Servo.o 

CC_DEPS += \
./libraries/Servo/Servo.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/Servo/%.o: ../libraries/Servo/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/rbtying/robot/firmware/Arduino-328P" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



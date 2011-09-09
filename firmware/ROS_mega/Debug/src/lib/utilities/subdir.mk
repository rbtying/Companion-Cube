################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/lib/utilities/FlexiTimer2.cc \
../src/lib/utilities/PID.cc \
../src/lib/utilities/RC_CTRL.cc 

OBJS += \
./src/lib/utilities/FlexiTimer2.o \
./src/lib/utilities/PID.o \
./src/lib/utilities/RC_CTRL.o 

CC_DEPS += \
./src/lib/utilities/FlexiTimer2.d \
./src/lib/utilities/PID.d \
./src/lib/utilities/RC_CTRL.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/utilities/%.o: ../src/lib/utilities/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/rbtying/robot/firmware/Arduino-2560/Debug" -I"/home/rbtying/robot/firmware/ROS_mega/src/lib" -I"/home/rbtying/robot/firmware/Arduino-2560" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


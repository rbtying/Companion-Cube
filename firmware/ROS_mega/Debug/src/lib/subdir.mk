################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/lib/CD74HC4067.cc \
../src/lib/Controller.cc \
../src/lib/FlexiTimer2.cc \
../src/lib/Gyro.cc \
../src/lib/MaxSonar.cc \
../src/lib/PID.cc \
../src/lib/Ping.cc \
../src/lib/PowerMonitor.cc \
../src/lib/RC_CTRL.cc \
../src/lib/Sabertooth.cc \
../src/lib/Sharp_IR.cc \
../src/lib/TSL1401.cc 

OBJS += \
./src/lib/CD74HC4067.o \
./src/lib/Controller.o \
./src/lib/FlexiTimer2.o \
./src/lib/Gyro.o \
./src/lib/MaxSonar.o \
./src/lib/PID.o \
./src/lib/Ping.o \
./src/lib/PowerMonitor.o \
./src/lib/RC_CTRL.o \
./src/lib/Sabertooth.o \
./src/lib/Sharp_IR.o \
./src/lib/TSL1401.o 

CC_DEPS += \
./src/lib/CD74HC4067.d \
./src/lib/Controller.d \
./src/lib/FlexiTimer2.d \
./src/lib/Gyro.d \
./src/lib/MaxSonar.d \
./src/lib/PID.d \
./src/lib/Ping.d \
./src/lib/PowerMonitor.d \
./src/lib/RC_CTRL.d \
./src/lib/Sabertooth.d \
./src/lib/Sharp_IR.d \
./src/lib/TSL1401.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/%.o: ../src/lib/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/rbtying/robot/firmware/Arduino-2560/Debug" -I"/home/rbtying/robot/firmware/Arduino-2560" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



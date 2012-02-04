################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/lib/sensors/imu/L3G4200D.cc \
../src/lib/sensors/imu/LSM303.cc 

OBJS += \
./src/lib/sensors/imu/L3G4200D.o \
./src/lib/sensors/imu/LSM303.o 

CC_DEPS += \
./src/lib/sensors/imu/L3G4200D.d \
./src/lib/sensors/imu/LSM303.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/sensors/imu/%.o: ../src/lib/sensors/imu/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/rbtying/robot/firmware/Arduino-2560/Debug" -I"/home/rbtying/robot/firmware/ROS_mega/src/ros_lib" -I"/home/rbtying/robot/firmware/ROS_mega/src/lib" -I"/home/rbtying/robot/firmware/Arduino-2560" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


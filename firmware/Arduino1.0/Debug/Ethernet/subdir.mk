################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Ethernet/Dhcp.cpp \
../Ethernet/Dns.cpp \
../Ethernet/Ethernet.cpp \
../Ethernet/EthernetClient.cpp \
../Ethernet/EthernetServer.cpp \
../Ethernet/EthernetUdp.cpp 

OBJS += \
./Ethernet/Dhcp.o \
./Ethernet/Dns.o \
./Ethernet/Ethernet.o \
./Ethernet/EthernetClient.o \
./Ethernet/EthernetServer.o \
./Ethernet/EthernetUdp.o 

CPP_DEPS += \
./Ethernet/Dhcp.d \
./Ethernet/Dns.d \
./Ethernet/Ethernet.d \
./Ethernet/EthernetClient.d \
./Ethernet/EthernetServer.d \
./Ethernet/EthernetUdp.d 


# Each subdirectory must supply rules for building sources it contributes
Ethernet/%.o: ../Ethernet/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/rbtying/robot/firmware/Arduino1.0" -I"/home/rbtying/robot/firmware/Arduino1.0/EEPROM" -I"/home/rbtying/robot/firmware/Arduino1.0/Ethernet" -I"/home/rbtying/robot/firmware/Arduino1.0/Firmata" -I"/home/rbtying/robot/firmware/Arduino1.0/LiquidCrystal" -I"/home/rbtying/robot/firmware/Arduino1.0/SD" -I"/home/rbtying/robot/firmware/Arduino1.0/Servo" -I"/home/rbtying/robot/firmware/Arduino1.0/SoftwareSerial" -I"/home/rbtying/robot/firmware/Arduino1.0/SPI" -I"/home/rbtying/robot/firmware/Arduino1.0/Stepper" -I"/home/rbtying/robot/firmware/Arduino1.0/Wire" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



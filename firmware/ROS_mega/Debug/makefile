################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/ros_lib/subdir.mk
-include src/lib/utilities/subdir.mk
-include src/lib/sensors/rangefinders/subdir.mk
-include src/lib/sensors/imu/subdir.mk
-include src/lib/sensors/subdir.mk
-include src/lib/motors/subdir.mk
-include src/lib/devices/subdir.mk
-include src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
ROS_mega.lss \

FLASH_IMAGE += \
ROS_mega.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: ROS_mega.elf secondary-outputs

# Tool invocations
ROS_mega.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++  -lc -lm -Wl,-Map,ROS_mega.map,--cref -Wl,--gc-sections -L"/home/rbtying/robot/firmware/Arduino1.0/Debug" -mmcu=atmega2560 -o"ROS_mega.elf"  $(OBJS) $(USER_OBJS) $(LIBS)  -lc
	@echo 'Finished building target: $@'
	@echo ' '

ROS_mega.lss: ROS_mega.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S ROS_mega.elf  >"ROS_mega.lss"
	@echo 'Finished building: $@'
	@echo ' '

ROS_mega.hex: ROS_mega.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex ROS_mega.elf  "ROS_mega.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: ROS_mega.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega2560 ROS_mega.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ELFS)$(FLASH_IMAGE)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) ROS_mega.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

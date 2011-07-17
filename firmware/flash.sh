#!/bin/bash

avrdude -pm328p -carduino -P/dev/ttyRoverFTDI -b57600 -Uflash:w:ROS_mini/Debug/ROS_mini.hex:a

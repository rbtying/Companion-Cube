#!/bin/bash
stty -F /dev/ttyRoverFTDI hupcl
avrdude -pm2560 -cstk500v2 -P/dev/ttyRoverFTDI -b115200 -Uflash:w:ROS_mega/Debug/ROS_mega.hex:a

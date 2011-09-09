/*
 * control_struct.h
 *
 *  Created on: Jul 17, 2011
 *      Author: rbtying
 */

#ifndef CONTROL_STRUCT_H_
#define CONTROL_STRUCT_H_

#include "libraries/Servo/Servo.h"
#include "devices/LCD.h"
#include "sensors/Battery.h"
#include "sensors/Gyro.h"
#include "sensors/Encoders.h"
#include "utilities/PID.h"
#include "motors/DualMotor.h"

struct control_data {
	Battery cpu_batt;
	Battery mot_batt;
	gyro_data yaw;
	Servo pan;
	Servo tilt;
	PID_params leftPID;
	PID_params rightPID;
	encoder_data leftEnc;
	encoder_data rightEnc;
	motor_data mot;
	LCD * lcd;
};

typedef struct control_data control_data;

#endif /* CONTROL_STRUCT_H_ */
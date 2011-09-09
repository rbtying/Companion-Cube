/*
 * control_opcodes.h
 *
 *  Created on: Sep 7, 2011
 *      Author: rbtying
 */

#ifndef CONTROL_OPCODES_H_
#define CONTROL_OPCODES_H_

#define	CTRL_OP_TOGGLE_POLL 0x01
#define CTRL_OP_SET_DRIVE 0x02
#define CTRL_OP_SET_SERVO 0x03
#define CTRL_OP_SET_PID 0x04
#define CTRL_OP_SET_CONV 0x05
#define CTRL_OP_SET_MOTOR 0x06
#define CTRL_OP_SET_LCD 0x07
#define CTRL_OP_GET_BATTERY_HUMAN 0x62 // b

#define MSG_OP_LIST 0x00
#define MSG_OP_LEFT_ENC 0x01
#define MSG_OP_RIGHT_ENC 0x02
#define MSG_OP_YAW_GYRO 0x03
#define MSG_OP_CPU_BAT 0x04
#define MSG_OP_MOTOR_BAT 0x05
#define MSG_OP_PAN_SERVO 0x06
#define MSG_OP_TILT_SERVO 0x07

#endif /* CONTROL_OPCODES_H_ */

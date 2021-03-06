/*
 * state_struct.h
 *
 *  Created on: Dec 1, 2011
 *      Author: rbtying
 */

#ifndef STATE_STRUCT_H_
#define STATE_STRUCT_H_

#define FLAG_MOTOR_ENABLED (0)

struct robot_state {
	uint8_t flag; // status flags
	uint8_t servo_pan_val; // pan servo angle, degrees
	uint8_t servo_tilt_val; // tilt servo angle, degrees
	int16_t batt_motor_voltage; // battery voltage * 100
	int16_t batt_motor_current; // battery current * 100
	int16_t gyro_yaw_rate; // gyro rate * 1000
	int16_t gyro_yaw_val; // gyro val * 1000
	int16_t enc_left_speed; // velocity (cm/s) * 100
	int16_t enc_right_speed; // velocity (cm/s) * 100
	int32_t enc_left_count; // count
	int32_t enc_right_count; // count
	int16_t enc_left_conv; // cmpercount * 1000
	int16_t enc_right_conv; // cmpercount * 1000
	int8_t motor_left_val; // value
	int8_t motor_right_val; // value
	int16_t pid_left_proportional; // * 100
	int16_t pid_left_integral; // * 100
	int16_t pid_left_derivative; // * 100
	int16_t pid_left_setpoint; // cm/s * 10
	int16_t pid_right_proportional; // * 100
	int16_t pid_right_integral; // * 100
	int16_t pid_right_derivative; // * 100
	int16_t pid_right_setpoint; // cm/s * 10
};
typedef struct robot_state robot_state;

#define STATE_STRUCT_SIZE 45

inline void byteToStateStruct(uint8_t * buf, robot_state * state) {
	state->flag = buf[0];
	state->servo_pan_val = buf[1];
	state->servo_tilt_val = buf[2];
	state->batt_motor_voltage = buf[3] << 8u | buf[4];
	state->batt_motor_current = buf[5] << 8u | buf[6];
	state->gyro_yaw_rate = buf[7] << 8u | buf[8];
	state->gyro_yaw_val = buf[9] << 8u | buf[10];
	state->enc_left_speed = buf[11] << 8u | buf[12];
	state->enc_right_speed = buf[13] << 8u | buf[14];
	state->enc_left_count = ((uint32_t) buf[15]) << 24u | ((uint32_t) buf[16])
			<< 16u | ((uint16_t) buf[17]) << 8u | buf[18];
	state->enc_right_count = ((uint32_t) buf[19]) << 24u | ((uint32_t) buf[20])
			<< 16u | ((uint16_t) buf[21]) << 8u | buf[22];
	state->enc_left_conv = buf[23] << 8u | buf[24];
	state->enc_right_conv = buf[25] << 8u | buf[26];
	state->motor_left_val = buf[27];
	state->motor_right_val = buf[28];
	state->pid_left_proportional = buf[29] << 8u | buf[30];
	state->pid_left_integral = buf[31] << 8u | buf[32];
	state->pid_left_derivative = buf[33] << 8u | buf[34];
	state->pid_left_setpoint = buf[35] << 8u | buf[36];
	state->pid_right_proportional = buf[37] << 8u | buf[38];
	state->pid_right_integral = buf[39] << 8u | buf[40];
	state->pid_right_derivative = buf[41] << 8u | buf[42];
	state->pid_right_setpoint = buf[43] << 8u | buf[44];
}

inline void stateStructToByte(robot_state * state, uint8_t * buf) {
	buf[0] = state->flag;
	buf[1] = state->servo_pan_val;
	buf[2] = state->servo_tilt_val;
	buf[3] = state->batt_motor_voltage >> 8u;
	buf[4] = state->batt_motor_voltage & 0xff;
	buf[5] = state->batt_motor_current >> 8u;
	buf[6] = state->batt_motor_current & 0xff;
	buf[7] = state->gyro_yaw_rate >> 8u;
	buf[8] = state->gyro_yaw_rate & 0xff;
	buf[9] = state->gyro_yaw_val >> 8u;
	buf[10] = state->gyro_yaw_val & 0xff;
	buf[11] = state->enc_left_speed >> 8u;
	buf[12] = state->enc_left_speed & 0xff;
	buf[13] = state->enc_right_speed >> 8u;
	buf[14] = state->enc_right_speed & 0xff;
	buf[15] = state->enc_left_count >> 24u;
	buf[16] = state->enc_left_count >> 16u;
	buf[17] = state->enc_left_count >> 8u;
	buf[18] = state->enc_left_count & 0xff;
	buf[19] = state->enc_right_count >> 24u;
	buf[20] = state->enc_right_count >> 16u;
	buf[21] = state->enc_right_count >> 8u;
	buf[22] = state->enc_right_count & 0xff;
	buf[23] = state->enc_left_conv >> 8u;
	buf[24] = state->enc_left_conv & 0xff;
	buf[25] = state->enc_right_conv >> 8u;
	buf[26] = state->enc_right_conv & 0xff;
	buf[27] = state->motor_left_val;
	buf[28] = state->motor_right_val;
	buf[29] = state->pid_left_proportional >> 8u;
	buf[30] = state->pid_left_proportional & 0xff;
	buf[31] = state->pid_left_integral >> 8u;
	buf[32] = state->pid_left_integral & 0xff;
	buf[33] = state->pid_left_derivative >> 8u;
	buf[34] = state->pid_left_derivative & 0xff;
	buf[35] = state->pid_left_setpoint >> 8u;
	buf[36] = state->pid_left_setpoint & 0xff;
	buf[37] = state->pid_right_proportional >> 8u;
	buf[38] = state->pid_right_proportional & 0xff;
	buf[39] = state->pid_right_integral >> 8u;
	buf[40] = state->pid_right_integral & 0xff;
	buf[41] = state->pid_right_derivative >> 8u;
	buf[42] = state->pid_right_derivative & 0xff;
	buf[43] = state->pid_right_setpoint >> 8u;
	buf[44] = state->pid_right_setpoint & 0xff;
}

#endif /* STATE_STRUCT_H_ */

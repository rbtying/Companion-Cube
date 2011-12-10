/*
 * Controller.cpp
 *
 *  Created on: Feb 21, 2011
 *      Author: rbtying
 */

#include "StateController.h"
#include "pins.h"

StateController::StateController(control_data * ctrl, HardwareSerial * hws) {
	m_ctrl = ctrl;
	m_hws = hws;

	comm = false;
	m_poll = true;
	m_lastUpdateTime = 0;
}

/**
 * Update the buffer
 */
void StateController::update() {
	if (m_poll && (m_lastPacketSendTime - millis() > CMD_PACKET_INTERVAL)) {
		StateController::controlToRobotState(m_ctrl, &m_state);
		StateController::sendDataPacket(&m_state);
	}

	while (m_hws->available()) {
		m_lastUpdateTime = millis();
		char c = m_hws->read();
		if (c == '<') {
			m_bufptr = 0;
		} else if (c == '>' || m_bufptr > STATE_STRUCT_SIZE) {
			readDataPacket(m_buf, m_bufptr, &m_state);
			m_bufptr = 0;
		} else {
			if (m_bufptr < STATE_STRUCT_SIZE)
				m_buf[m_bufptr] = c;
			++m_bufptr;
		}
	}
	comm = (millis() - m_lastUpdateTime) <= CMD_TIMEOUT;
}

/**
 * Send a full data packet
 */
void StateController::sendDataPacket(robot_state * state) {
	uint8_t buf[STATE_STRUCT_SIZE];

	stateStructToByte(state, buf);

	for (uint8_t i = 0; i < STATE_STRUCT_SIZE; i++) {
		if (buf[i] == '<' || buf[i] == '>') {
			buf[i] = '=';
		}
	}

	m_hws->write('<');
	m_hws->write(buf, STATE_STRUCT_SIZE);
	m_hws->write('>');

}

void StateController::readDataPacket(uint8_t * contents, uint8_t length,
		robot_state * state) {
	if (length == STATE_STRUCT_SIZE) {
		byteToStateStruct(contents, state);
		robotStateToControl(state, m_ctrl);
	}
}

void StateController::robotStateToControl(robot_state * state,
		control_data * ctrl) {
	ctrl->pan.write(state->servo_pan_val);
	ctrl->tilt.write(state->servo_tilt_val);
	// read only values
	//	state->batt_motor_voltage * 0.01;
	//	state->batt_motor_current * 0.01;
	//	state->gyro_yaw_rate * 0.001;
	//	state->gyro_yaw_val * 0.001;
	//	state->enc_left_speed * 0.01;
	//	state->enc_right_speed * 0.01;
	//	state->enc_left_count;
	//	state->enc_right_count;
	ctrl->leftEnc.cmPerCount = state->enc_left_conv * 0.0001;
	ctrl->rightEnc.cmPerCount = state->enc_right_conv * 0.0001;
	//	more read only values
	//	state->motor_left_val;
	//	state->motor_right_val;
	ctrl->leftPID.proportional = state->pid_left_proportional * 0.01;
	ctrl->leftPID.integral = state->pid_left_integral * 0.01;
	ctrl->leftPID.derivative = state->pid_left_derivative * 0.01;
	ctrl->leftPID.set = state->pid_left_setpoint * 0.1;
	ctrl->rightPID.proportional = state->pid_right_proportional * 0.01;
	ctrl->rightPID.integral = state->pid_right_integral * 0.01;
	ctrl->rightPID.derivative = state->pid_right_derivative * 0.01;
	ctrl->rightPID.set = state->pid_right_setpoint * 0.1;
	ctrl->enabled = state->flag & (1 << FLAG_MOTOR_ENABLED);
}

void StateController::controlToRobotState(control_data * ctrl,
		robot_state * state) {
	state->servo_pan_val = ctrl->pan.read();
	state->servo_tilt_val = ctrl->tilt.read();
	state->batt_motor_voltage = ctrl->mot_batt.getVoltage() * 100;
	state->batt_motor_current = ctrl->mot_batt.getCurrent() * 100;
	state->gyro_yaw_rate = ctrl->yaw.rate * 1000;
	state->gyro_yaw_val = ctrl->yaw.val * 1000;
	state->enc_left_speed = ctrl->leftEnc.velocity * 100;
	state->enc_right_speed = ctrl->rightEnc.velocity * 100;
	state->enc_left_count = ctrl->leftEnc.count;
	state->enc_right_count = ctrl->leftEnc.count;
	//  write only values
	//	state->enc_left_conv * 1000;
	//	state->enc_right_conv * 1000;
	state->motor_left_val = ctrl->mot.leftSpeed;
	state->motor_right_val = ctrl->mot.rightSpeed;
	state->flag = ctrl->enabled << FLAG_MOTOR_ENABLED;
	//  more write only values
	//	state->pid_left_proportional * 100;
	//	state->pid_left_integral * 100;
	//	state->pid_left_derivative * 100;
	//	state->pid_left_setpoint * 0.01;
	//	state->pid_right_proportional * 100;
	//	state->pid_right_integral * 100;
	//	state->pid_right_derivative * 100;
	//	state->pid_right_setpoint * 0.01;
}

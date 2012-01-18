/*
 * Controller.h
 *
 *  Created on: Feb 21, 2011
 *      Author: rbtying
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <WProgram.h>
#include "control_struct.h"
#include "state_struct.h"
#include "sensors/Battery.h"
#include "libraries/Servo/Servo.h"

#define CMD_PACKET_INTERVAL 40 // 25 Hz
#define CMD_TIMEOUT 1000

class StateController {
public:
	StateController(control_data * ctrl, HardwareSerial * hws);
	void update();
	void robotStateToControl(robot_state * state, control_data * ctrl);
	void controlToRobotState(control_data * ctrl, robot_state * state);

	bool comm;

private:
	control_data * m_ctrl;
	robot_state m_state;
	HardwareSerial * m_hws;
	uint8_t m_buf[STATE_STRUCT_SIZE];
	uint8_t m_bufptr;

	bool m_poll;
	unsigned long m_lastUpdateTime, m_lastPacketSendTime;
	void sendDataPacket(robot_state * state);
	void
	readDataPacket(uint8_t * contents, uint8_t length, robot_state * state);
};

#endif /* CONTROLLER_H_ */
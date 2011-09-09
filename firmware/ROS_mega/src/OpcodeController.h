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
#include "control_opcodes.h"
#include "sensors/Battery.h"
#include "libraries/Servo/Servo.h"

#define CMD_PACKET_INTERVAL 40 // 25 Hz
#define CMD_TIMEOUT 1000

class OpcodeController {
public:
	OpcodeController(control_data * ctrl, HardwareSerial * hws);
	void update();

	bool comm;

private:
	control_data * m_ctrl;
	HardwareSerial * m_hws;

	bool m_poll;
	unsigned long m_lastUpdateTime, m_lastPacketSendTime;
	void sendDataPacket();
	void sendMessage(uint8_t op, uint8_t * contents, uint8_t length);
	void sendEncoderMessage(uint8_t op, encoder_data * d, uint8_t out);
	void sendGyroMessage(uint8_t op, gyro_data * d);
	void sendBatteryMessage(uint8_t op, Battery * b);
	void sendServoMessage(uint8_t op, uint8_t angle);
	uint8_t checksum(uint8_t op, uint8_t * contents, uint8_t length);
	char nextByte(unsigned long timeout);
};

#endif /* CONTROLLER_H_ */

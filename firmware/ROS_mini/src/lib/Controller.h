/*
 * Controller.h
 *
 *  Created on: Feb 21, 2011
 *      Author: rbtying
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "WProgram.h"
#include "PowerMonitor.h"
#include "control_struct.h"
#include "libraries/Servo/Servo.h"

#define CMD_BUFFER_SIZE 8
#define CMD_PACKET_INTERVAL 40 // 25 Hz
#define CMD_STARTCHAR ':'
#define CMD_ENDCHAR '!'
#define CMD_TIMEOUT 1000

#define SIGN_LEFT_VEL 0
#define SIGN_RIGHT_VEL 1
#define SIGN_YAW_RATE 2
#define SIGN_YAW_VAL 3
#define SIGN_LEFT_ENC 4
#define SIGN_RIGHT_ENC 5

// this thing is little endian
struct statusMessage {
	uint8_t leadingChar;
	uint8_t sign; // signs for the next few items
	uint16_t leftSpeed; // in mm
	uint16_t rightSpeed; // in mm
	int8_t leftOutSpeed; // actual power out
	int8_t rightOutSpeed; // actual power out
	uint32_t leftCount; // in encoder counts
	uint32_t rightCount; // in encoder counts
	uint16_t yawRate; // in radians/s * 1000
	uint16_t yawVal; // in radians * 1000
	int16_t cpu_voltage; // in volts * 100
	int16_t cpu_current; // in amperes * 100
	int16_t mot_voltage; // in volts * 100
	int16_t mot_current; // in amperes * 100
	uint8_t panAngle; // in degrees
	uint8_t tiltAngle; // in degrees
	uint8_t endingChar;
};

typedef statusMessage statusMessage;

class Controller {
public:
	Controller(control_data * ctrl);
	void update();
	void printInfo();

	bool comm;

private:
	control_data * m_ctrl;

	bool m_poll;
	unsigned long m_lastUpdateTime, m_lastPacketSendTime;
	char m_buf[CMD_BUFFER_SIZE];
	statusMessage m_stat;
	uint8_t m_bufPtr;
	uint8_t m_dataPacket[sizeof(statusMessage)];
	void processCommand();
	void sendDataPacket();
	char nextByte(unsigned long timeout);
	void flush();
};

#endif /* CONTROLLER_H_ */

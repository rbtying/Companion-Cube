/*
 * Controller.cpp
 *
 *  Created on: Feb 21, 2011
 *      Author: rbtying
 */

#include "Controller.h"

#include "PowerMonitor.h"

Controller::Controller(control_data * ctrl) {
	m_ctrl = ctrl;

	m_bufPtr = 0;
	comm = false;
	m_poll = true;
	m_lastUpdateTime = 0;
}

/**
 * Update the buffer
 */
void Controller::update() {
	if (m_poll && (m_lastPacketSendTime - millis() > CMD_PACKET_INTERVAL)) {
		Controller::sendDataPacket();
	}
	while (Serial.available()) {
		m_lastUpdateTime = millis();
		char c = Serial.read();
		if (c == CMD_STARTCHAR) {
			Controller::flush();
		} else if (c == CMD_ENDCHAR || m_bufPtr >= CMD_BUFFER_SIZE - 1) {
			m_buf[m_bufPtr] = '\0';
			Controller::processCommand();
		} else {
			m_buf[m_bufPtr] = c;
			m_bufPtr++;
		}
	}

	comm = (millis() - m_lastUpdateTime) <= CMD_TIMEOUT;
}
void Controller::processCommand() {
	if (strstr(m_buf, "ss") != NULL) {
		m_poll = nextByte(250);
	} else if (strstr(m_buf, "sDRV") != NULL) {
		byte msg[4];
		for (uint8_t i = 0; i < 4; i++) {
			msg[i] = nextByte(250);
		}
		m_ctrl->leftPID.set = ((msg[0] << 8) | msg[1]) * 0.1; // mm/s -> cm/s
		m_ctrl->rightPID.set = ((msg[2] << 8) | msg[3]) * 0.1; // mm/s -> cm/s
	} else if (strstr(m_buf, "sSER") != NULL) {
		byte msg[2];
		msg[0] = nextByte(100);
		msg[1] = nextByte(100);
		msg[0] = constrain(msg[0], 0, 180);
		msg[1] = constrain(msg[1], 0, 180);
		m_ctrl->pan.write(msg[0]);
		m_ctrl->tilt.write(msg[1]);
	} else if (strstr(m_buf, "sPID") != NULL) {
		byte msg[12];
		for (uint8_t i = 0; i < 12; i++) {
			msg[i] = nextByte(250);
		}
		m_ctrl->leftPID.proportional = ((msg[0] << 8) | msg[1]) * 0.01;
		m_ctrl->leftPID.integral = ((msg[2] << 8) | msg[3]) * 0.01;
		m_ctrl->leftPID.derivative = ((msg[4] << 8) | msg[5]) * 0.01;

		m_ctrl->rightPID.proportional = ((msg[6] << 8) | msg[7]) * 0.01;
		m_ctrl->rightPID.integral = ((msg[8] << 8) | msg[9]) * 0.01;
		m_ctrl->rightPID.derivative = ((msg[10] << 8) | msg[11]) * 0.01;
	} else if (strstr(m_buf, "sCONV") != NULL) {
		byte msg[4];
		for (uint8_t i = 0; i < 4; i++) {
			msg[i] = nextByte(250);
		}
		m_ctrl->leftEnc.cmPerCount = ((msg[0] << 8) | msg[1]) * 0.0001;
		m_ctrl->rightEnc.cmPerCount = ((msg[2] << 8) | msg[3]) * 0.0001;
	} else if (strstr(m_buf, "sMOT")) {
		int8_t msg[2];
		msg[0] = nextByte(250);
		msg[1] = nextByte(250);
		m_ctrl->mot.leftSpeed = msg[0];
		m_ctrl->mot.rightSpeed = msg[1];
	} else if (strstr(m_buf, "gPIDH") != NULL) {
		Serial.print("left PID: [");
		Serial.print(m_ctrl->leftPID.proportional, DEC);
		Serial.print(", ");
		Serial.print(m_ctrl->leftPID.integral, DEC);
		Serial.print(", ");
		Serial.print(m_ctrl->leftPID.derivative, DEC);
		Serial.print("] right PID: [");
		Serial.print(m_ctrl->rightPID.proportional, DEC);
		Serial.print(", ");
		Serial.print(m_ctrl->rightPID.integral, DEC);
		Serial.print(", ");
		Serial.print(m_ctrl->rightPID.derivative, DEC);
		Serial.print("]\r\n");
	} else if (strstr(m_buf, "gENC") != NULL) {
		int16_t leftSpeed_mm = (int16_t) (m_ctrl->leftPID.input * 10); // cm to mm & truncate
		int16_t rightSpeed_mm = (int16_t) (m_ctrl->rightPID.input * 10); // cm to mm & truncate

		m_dataPacket[0] = '<';
		m_dataPacket[1] = highByte(leftSpeed_mm);
		m_dataPacket[2] = lowByte(leftSpeed_mm);
		m_dataPacket[3] = highByte(rightSpeed_mm);
		m_dataPacket[4] = lowByte(rightSpeed_mm);
		m_dataPacket[5] = (m_dataPacket[1] + m_dataPacket[2] + m_dataPacket[3]
				+ m_dataPacket[4]) & 0x7f;
		m_dataPacket[6] = '>';

		Serial.write(m_dataPacket, 7);
	} else if (strstr(m_buf, "gGYR") != NULL) {
		int16_t yawRate = (int16_t) (m_ctrl->yaw.rate * 100);
		int16_t yawVal = (int16_t) (m_ctrl->yaw.val * 100);

		m_dataPacket[0] = '<';
		m_dataPacket[1] = highByte(yawRate);
		m_dataPacket[2] = lowByte(yawRate);
		m_dataPacket[3] = highByte(yawVal);
		m_dataPacket[4] = lowByte(yawVal);
		m_dataPacket[5] = (m_dataPacket[1] + m_dataPacket[2] + m_dataPacket[3]
				+ m_dataPacket[4]) & 0x7f;
		m_dataPacket[6] = '>';

		Serial.write(m_dataPacket, 7);
	} else if (strstr(m_buf, "gBTY") != NULL) {
		int16_t batVal = (int16_t) (m_ctrl->cpu_batt.getVoltage() * 100);
		int16_t curVal = (int16_t) (m_ctrl->cpu_batt.getCurrent() * 100);

		m_dataPacket[0] = '<';
		m_dataPacket[1] = highByte(batVal);
		m_dataPacket[2] = lowByte(batVal);
		m_dataPacket[3] = highByte(curVal);
		m_dataPacket[4] = lowByte(curVal);
		m_dataPacket[5] = (m_dataPacket[1] + m_dataPacket[2] + m_dataPacket[3]
				+ m_dataPacket[4]) & 0x7f;
		m_dataPacket[6] = '>';

		Serial.write(m_dataPacket, 7);
	} else if (strstr(m_buf, "gSER") != NULL) {
		uint8_t panAngle = (uint8_t) m_ctrl->pan.read();
		uint8_t tiltAngle = (uint8_t) m_ctrl->tilt.read();

		m_dataPacket[0] = '<';
		m_dataPacket[1] = panAngle & 0xFF;
		m_dataPacket[2] = tiltAngle & 0xFF;
		m_dataPacket[3] = (m_dataPacket[1] + m_dataPacket[2]) & 0x7f;
		m_dataPacket[4] = '>';

		Serial.write(m_dataPacket, 5);
	} else if (strstr(m_buf, "gLSP") != NULL) {
		Serial.println(m_ctrl->leftPID.set, DEC);
	} else if (strstr(m_buf, "gRSP") != NULL) {
		Serial.println(m_ctrl->rightPID.set, DEC);
	} else if (strstr(m_buf, "gLMS") != NULL) {
		Serial.println(m_ctrl->leftPID.input, DEC);
	} else if (strstr(m_buf, "gRMS") != NULL) {
		Serial.println(m_ctrl->rightPID.input, DEC);
	} else if (strstr(m_buf, "gBAT") != NULL) {
		Serial.print(m_ctrl->cpu_batt.getVoltage(), DEC);
		Serial.print(" volts, ");
		Serial.print(m_ctrl->cpu_batt.getCurrent(), DEC);
		Serial.print(" amps\r\n");
	} else {
		Serial.println("INVALID COMMAND");
	}

	Controller::flush();
}

/**
 * Send a full data packet
 */
void Controller::sendDataPacket() {
	m_stat.leadingChar = '<';
	m_stat.endingChar = '>';
	m_stat.leftSpeed = (uint16_t) (abs(m_ctrl->leftEnc.velocity * 100)); // cm to mm & truncate
	m_stat.rightSpeed = (uint16_t) (abs(m_ctrl->rightEnc.velocity * 100)); // cm to mm & truncate
	m_stat.leftOutSpeed = m_ctrl->mot.leftSpeed & 0xff;
	m_stat.rightOutSpeed = m_ctrl->mot.rightSpeed & 0xff;
	m_stat.leftCount = abs(m_ctrl->leftEnc.count);
	m_stat.rightCount = abs(m_ctrl->rightEnc.count);
	m_stat.yawRate = (uint16_t) (abs(m_ctrl->yaw.rate) * 1000);
	m_stat.yawVal = (uint16_t) (abs(m_ctrl->yaw.val) * 1000);
	m_stat.cpu_voltage = (int16_t) (m_ctrl->cpu_batt.getVoltage() * 100);
	m_stat.cpu_current = (int16_t) (m_ctrl->cpu_batt.getCurrent() * 100);
	m_stat.mot_voltage = (int16_t) (m_ctrl->mot_batt.getVoltage() * 100);
	m_stat.mot_current = (int16_t) (m_ctrl->mot_batt.getCurrent() * 100);
	m_stat.panAngle = (uint8_t) (m_ctrl->pan.read());
	m_stat.tiltAngle = (uint8_t) (m_ctrl->tilt.read());
	m_stat.sign = 0;
	m_stat.sign |= ((m_ctrl->leftEnc.velocity >= 0) << SIGN_LEFT_VEL);
	m_stat.sign |= ((m_ctrl->rightEnc.velocity >= 0) << SIGN_RIGHT_VEL);
	m_stat.sign |= ((m_ctrl->yaw.rate >= 0) << SIGN_YAW_RATE);
	m_stat.sign |= ((m_ctrl->yaw.val >= 0) << SIGN_YAW_VAL);
	m_stat.sign |= ((m_ctrl->leftEnc.count >= 0) << SIGN_LEFT_ENC);
	m_stat.sign |= ((m_ctrl->rightEnc.count >= 0) << SIGN_RIGHT_ENC);

	m_dataPacket[0] = m_stat.leadingChar;
	m_dataPacket[1] = m_stat.sign & 0xff;
	m_dataPacket[2] = m_stat.leftSpeed >> 8u;
	m_dataPacket[3] = m_stat.leftSpeed & 0xff;
	m_dataPacket[4] = m_stat.rightSpeed >> 8u;
	m_dataPacket[5] = m_stat.rightSpeed & 0xff;
	m_dataPacket[6] = m_stat.leftOutSpeed & 0xff;
	m_dataPacket[7] = m_stat.rightOutSpeed & 0xff;
	m_dataPacket[8] = m_stat.leftCount >> 24u;
	m_dataPacket[9] = m_stat.leftCount >> 16u;
	m_dataPacket[10] = m_stat.leftCount >> 8u;
	m_dataPacket[11] = m_stat.leftCount & 0xff;
	m_dataPacket[12] = m_stat.rightCount >> 24u;
	m_dataPacket[13] = m_stat.rightCount >> 16u;
	m_dataPacket[14] = m_stat.rightCount >> 8u;
	m_dataPacket[15] = m_stat.rightCount & 0xff;
	m_dataPacket[16] = m_stat.yawRate >> 8u;
	m_dataPacket[17] = m_stat.yawRate & 0xff;
	m_dataPacket[18] = m_stat.yawVal >> 8u;
	m_dataPacket[19] = m_stat.yawVal & 0xff;
	m_dataPacket[20] = m_stat.cpu_voltage >> 8u;
	m_dataPacket[21] = m_stat.cpu_voltage & 0xff;
	m_dataPacket[22] = m_stat.cpu_current >> 8u;
	m_dataPacket[23] = m_stat.cpu_current & 0xff;
	m_dataPacket[24] = m_stat.mot_voltage >> 8u;
	m_dataPacket[25] = m_stat.mot_current & 0xff;
	m_dataPacket[26] = m_stat.tiltAngle & 0xff;
	m_dataPacket[27] = m_stat.panAngle & 0xff;
	m_dataPacket[28] = m_stat.endingChar;

	Serial.write(m_dataPacket, sizeof(statusMessage));
}

/**
 * Clear the buffer and reset the pointer
 */
void Controller::flush() {
	m_bufPtr = 0;
	for (uint8_t i = 0; i < CMD_BUFFER_SIZE; i++)
		m_buf[i] = ' ';
}

/**
 * Request a byte from Serial
 */
char Controller::nextByte(unsigned long timeout) {
	unsigned long timeOutTime = millis() + timeout;
	while (millis() < timeOutTime && !Serial.available()) {
		// do nothing
	}
	if (Serial.available())
		return Serial.read();
	else
		return 0x00;
}

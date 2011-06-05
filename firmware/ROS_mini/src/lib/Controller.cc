/*
 * Controller.cpp
 *
 *  Created on: Feb 21, 2011
 *      Author: rbtying
 */

#include "Controller.h"

#include "PowerMonitor.h"

Controller::Controller(double *lSet, double *rSet, double *lMeas,
		double *rMeas, long *lEnc, long *rEnc, PowerMonitor *batt,
		double *yawRate, double *yawVal, Servo *pan, Servo *tilt) {
	m_lSet = lSet;
	m_rSet = rSet;
	m_lMeas = lMeas;
	m_rMeas = rMeas;
	m_batt = batt;
	m_lEnc = lEnc;
	m_rEnc = rEnc;
	m_yawRate = yawRate;
	m_yawVal = yawVal;
	m_tilt = tilt;
	m_pan = pan;

	m_bufPtr = 0;

	comm = false;

	m_lastUpdateTime = 0;
}

/**
 * Update the buffer
 */
void Controller::update() {
	if (m_lastPacketSendTime - millis() > CMD_PACKET_INTERVAL) {
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
	if (strstr(m_buf, "sDRV") != NULL) {
		byte msg[4];
		for (uint8_t i = 0; i < 4; i++) {
			msg[i] = nextByte(250);
		}
		*m_lSet = ((msg[0] << 8) | msg[1]) * 0.1; // mm/s -> cm/s
		*m_rSet = ((msg[2] << 8) | msg[3]) * 0.1; // mm/s -> cm/s
	} else if (strstr(m_buf, "gENC") != NULL) {
		int16_t leftSpeed_mm = (int16_t) (*m_lMeas * 10); // cm to mm & truncate
		int16_t rightSpeed_mm = (int16_t) (*m_rMeas * 10); // cm to mm & truncate

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
		int16_t yawRate = (int16_t) (*m_yawRate * 100);
		int16_t yawVal = (int16_t) (*m_yawVal * 100);

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
		int16_t batVal = (int16_t) ((m_batt->getVoltage()) * 100);
		int16_t curVal = (int16_t) ((m_batt->getCurrent()) * 100);

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
		uint8_t panAngle = (uint8_t) m_pan->read();
		uint8_t tiltAngle = (uint8_t) m_tilt->read();

		m_dataPacket[0] = '<';
		m_dataPacket[1] = panAngle & 0xFF;
		m_dataPacket[2] = tiltAngle & 0xFF;
		m_dataPacket[3] = (m_dataPacket[1] + m_dataPacket[2]) & 0x7f;
		m_dataPacket[4] = '>';

		Serial.write(m_dataPacket, 5);
	} else if (strstr(m_buf, "sSER") != NULL) {
		byte msg[2];
		msg[0] = nextByte(100);
		msg[1] = nextByte(100);
		msg[0] = constrain(msg[0], 0, 180);
		msg[1] = constrain(msg[1], 0, 180);
		m_pan->write(msg[0]);
		m_tilt->write(msg[1]);
	} else if (strstr(m_buf, "gCNT") != NULL) {
		m_dataPacket[0] = '<';
		m_dataPacket[1] = *m_lEnc >> 24;
		m_dataPacket[2] = *m_lEnc >> 16;
		m_dataPacket[3] = *m_lEnc >> 8;
		m_dataPacket[4] = *m_lEnc & 0xFF;
		m_dataPacket[5] = *m_rEnc >> 24;
		m_dataPacket[6] = *m_rEnc >> 16;
		m_dataPacket[7] = *m_rEnc >> 8;
		m_dataPacket[8] = *m_rEnc & 0xFF;
		m_dataPacket[9] = '>';

		Serial.write(m_dataPacket, 10);
	} else if (strstr(m_buf, "gLSP") != NULL) {
		Serial.println(*m_lSet, DEC);
	} else if (strstr(m_buf, "gRSP") != NULL) {
		Serial.println(*m_rSet, DEC);
	} else if (strstr(m_buf, "gLMS") != NULL) {
		Serial.println(*m_lMeas, DEC);
	} else if (strstr(m_buf, "gRMS") != NULL) {
		Serial.println(*m_rMeas, DEC);
	} else if (strstr(m_buf, "gBAT") != NULL) {
		Serial.print(m_batt->getVoltage(), DEC);
		Serial.print(" volts, ");
		Serial.print(m_batt->getCurrent(), DEC);
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
	uint16_t leftSpeed_mm = (uint16_t) (abs(*m_lMeas * 100)); // cm to mm & truncate
	uint16_t rightSpeed_mm = (uint16_t) (abs(*m_rMeas * 100)); // cm to mm & truncate
	uint16_t yawRate = (uint16_t) (*m_yawRate * 1000);
	uint16_t yawVal = (uint16_t) (*m_yawVal * 1000);
	uint8_t signs;
	signs |= (*m_lMeas >= 0) << 0;
	signs |= (*m_rMeas >= 0) << 1;
	signs |= (*m_yawRate >= 0) << 2;
	signs |= (*m_yawVal >= 0) << 3;
	int16_t batVal = (int16_t) ((m_batt->getVoltage()) * 100);
	int16_t curVal = (int16_t) ((m_batt->getCurrent()) * 100);
	uint8_t panAngle = (uint8_t) m_pan->read();
	uint8_t tiltAngle = (uint8_t) m_tilt->read();

	m_dataPacket[0] = '<';
	m_dataPacket[1] = highByte(leftSpeed_mm);
	m_dataPacket[2] = lowByte(leftSpeed_mm);
	m_dataPacket[3] = highByte(rightSpeed_mm);
	m_dataPacket[4] = lowByte(rightSpeed_mm);
	m_dataPacket[5] = highByte(yawRate);
	m_dataPacket[6] = lowByte(yawRate);
	m_dataPacket[7] = highByte(yawVal);
	m_dataPacket[8] = lowByte(yawVal);
	m_dataPacket[9] = signs;
	m_dataPacket[10] = highByte(batVal);
	m_dataPacket[11] = lowByte(batVal);
	m_dataPacket[12] = highByte(curVal);
	m_dataPacket[13] = lowByte(curVal);
	m_dataPacket[14] = panAngle & 0xFF;
	m_dataPacket[15] = tiltAngle & 0xFF;
	m_dataPacket[16] = '>';

	Serial.write(m_dataPacket, CMD_PACKET_SIZE);
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

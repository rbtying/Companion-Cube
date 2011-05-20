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
	byte m[11];

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

		m[0] = '<';
		m[1] = highByte(leftSpeed_mm);
		m[2] = lowByte(leftSpeed_mm);
		m[3] = highByte(rightSpeed_mm);
		m[4] = lowByte(rightSpeed_mm);
		m[5] = (m[1] + m[2] + m[3] + m[4]) & 0x7f;
		m[6] = '>';

		Serial.write(m, 7);
	} else if (strstr(m_buf, "gGYR") != NULL) {
		int16_t yawRate = (int16_t) (*m_yawRate * 100);
		int16_t yawVal = (int16_t) (*m_yawVal * 100);

		m[0] = '<';
		m[1] = highByte(yawRate);
		m[2] = lowByte(yawRate);
		m[3] = highByte(yawVal);
		m[4] = lowByte(yawVal);
		m[5] = (m[1] + m[2] + m[3] + m[4]) & 0x7f;
		m[6] = '>';

		Serial.write(m, 7);
	} else if (strstr(m_buf, "gBTY") != NULL) {
		int16_t batVal = (int16_t) ((m_batt->getVoltage()) * 100);
		int16_t curVal = (int16_t) ((m_batt->getCurrent()) * 100);

		m[0] = '<';
		m[1] = highByte(batVal);
		m[2] = lowByte(batVal);
		m[3] = highByte(curVal);
		m[4] = lowByte(curVal);
		m[5] = (m[1] + m[2] + m[3] + m[4]) & 0x7f;
		m[6] = '>';

		Serial.write(m, 7);
	} else if (strstr(m_buf, "gSER") != NULL) {
		uint8_t panAngle = (uint8_t) m_pan->read();
		uint8_t tiltAngle = (uint8_t) m_tilt->read();

		m[0] = '<';
		m[1] = panAngle & 0xFF;
		m[2] = tiltAngle & 0xFF;
		m[3] = (m[1] + m[2]) & 0x7f;
		m[4] = '>';

		Serial.write(m, 5);
	} else if (strstr(m_buf, "sSER") != NULL) {
		byte msg[2];
		msg[0] = nextByte(100);
		msg[1] = nextByte(100);
		msg[0] = constrain(msg[0], 0, 180);
		msg[1] = constrain(msg[1], 0, 180);
		m_pan->write(msg[0]);
		m_tilt->write(msg[1]);
	} else if (strstr(m_buf, "gCNT") != NULL) {
		m[0] = '<';
		m[1] = *m_lEnc >> 24;
		m[2] = *m_lEnc >> 16;
		m[3] = *m_lEnc >> 8;
		m[4] = *m_lEnc & 0xFF;
		m[5] = *m_rEnc >> 24;
		m[6] = *m_rEnc >> 16;
		m[7] = *m_rEnc >> 8;
		m[8] = *m_rEnc & 0xFF;
		m[9] = '>';

		Serial.write(m, 10);
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

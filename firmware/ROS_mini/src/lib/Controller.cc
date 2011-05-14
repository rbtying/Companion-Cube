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

	newSpd = true;
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
	if (strstr(m_buf, "sDRV") != NULL) {
		byte msg[4];
		for (uint8_t i = 0; i < 4; i++) {
			msg[i] = nextByte(250);
		}
		*m_lSet = ((msg[0] << 8) | msg[1]) * 0.1; // mm/s -> cm/s
		*m_rSet = ((msg[2] << 8) | msg[3]) * 0.1; // mm/s -> cm/s
		newSpd = true;
	} else if (strstr(m_buf, "gENC") != NULL) {
		int16_t leftSpeed_mm = (int16_t)(*m_lMeas * 10); // cm to mm & truncate
		int16_t rightSpeed_mm = (int16_t)(*m_rMeas * 10); // cm to mm & truncate
		Serial.print("<");
		Serial.print(highByte(leftSpeed_mm), BYTE); // print high byte
		Serial.print(lowByte(leftSpeed_mm), BYTE); // print low byte
		Serial.print(highByte(rightSpeed_mm), BYTE); // print high byte
		Serial.print(lowByte(rightSpeed_mm), BYTE); // print low byte
		Serial.print(">");
	} else if (strstr(m_buf, "gGYR") != NULL) {
		int16_t yawRate = (int16_t)(*m_yawRate * 100);
		int16_t yawVal = (int16_t)(*m_yawVal * 100);
		Serial.print("<");
		Serial.print(highByte(yawRate), BYTE);
		Serial.print(lowByte(yawRate), BYTE);
		Serial.print(highByte(yawVal), BYTE);
		Serial.print(lowByte(yawVal), BYTE);
		Serial.print(">");
	} else if (strstr(m_buf, "gBTY") != NULL) {
		int16_t batVal = (int16_t)((m_batt->getVoltage()) * 100);
		int16_t curVal = (int16_t)((m_batt->getCurrent()) * 100);
		Serial.print("<");
		Serial.print(highByte(batVal), BYTE);
		Serial.print(lowByte(batVal), BYTE);
		Serial.print(highByte(curVal));
		Serial.print(lowByte(curVal));
		Serial.print(">");
	} else if (strstr(m_buf, "gSER") != NULL) {
		uint8_t panAngle = (uint8_t) m_pan->read();
		uint8_t tiltAngle = (uint8_t) m_tilt->read();
		Serial.print("<");
		Serial.print(panAngle & 0xFF, BYTE);
		Serial.print(tiltAngle & 0xFF, BYTE);
		Serial.print(">");
	} else if (strstr(m_buf, "sSER") != NULL) {
		byte msg[2];
		msg[0] = nextByte(100);
		msg[1] = nextByte(100);
		msg[0] = constrain(msg[0], 0, 180);
		msg[1] = constrain(msg[1], 0, 180);
		m_pan->write(msg[0]);
		m_tilt->write(msg[1]);
	} else if (strstr(m_buf, "gSEN") != NULL) {
		printInfo();
	} else if (strstr(m_buf, "gCNT") != NULL) {
		Serial.print("<");
		Serial.print((byte)(*m_lEnc >> 24), BYTE);
		Serial.print((byte)(*m_lEnc >> 16), BYTE);
		Serial.print((byte)(*m_lEnc >> 8), BYTE);
		Serial.print((byte)(*m_lEnc), BYTE);

		Serial.print((byte)(*m_rEnc >> 24), BYTE);
		Serial.print((byte)(*m_rEnc >> 16), BYTE);
		Serial.print((byte)(*m_rEnc >> 8), BYTE);
		Serial.print((byte)(*m_rEnc), BYTE);
		Serial.print(">");
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

void Controller::printInfo() {
	Serial.print("<");
	int16_t leftSpeed_mm = (int16_t)(*m_lMeas * 10); // cm to mm & truncate
	int16_t rightSpeed_mm = (int16_t)(*m_rMeas * 10); // cm to mm & truncate
	int16_t batVal = (int16_t)((m_batt->getVoltage()) * 100);
	int16_t curVal = (int16_t)((m_batt->getCurrent()) * 100);
	int16_t leftSet_mm = (int16_t)(*m_lSet * 10);
	int16_t rightSet_mm = (int16_t)(*m_rSet * 10);
	Serial.print(highByte(leftSpeed_mm), BYTE); // print high byte
	Serial.print(lowByte(leftSpeed_mm), BYTE); // print low byte
	Serial.print(highByte(rightSpeed_mm), BYTE); // print high byte
	Serial.print(lowByte(rightSpeed_mm), BYTE); // print low byte
	Serial.print(highByte(batVal), BYTE);
	Serial.print(lowByte(batVal), BYTE);
	Serial.print(highByte(curVal), BYTE);
	Serial.print(lowByte(curVal), BYTE);
	Serial.print(highByte(leftSet_mm));
	Serial.print(lowByte(leftSet_mm));
	Serial.print(highByte(rightSet_mm));
	Serial.print(lowByte(rightSet_mm));
	Serial.print(">");
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

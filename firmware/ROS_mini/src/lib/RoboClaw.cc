/*
 * RoboClaw.cpp
 *
 *  Created on: Feb 20, 2011
 *      Author: rbtying
 */

#include "RoboClaw.h"
#include "WProgram.h"

//#define DEBUG
#define NEED_HACK

/**
 * Construct a RoboClaw and set the resolution
 */
RoboClaw::RoboClaw(uint8_t rxPin, uint8_t txPin, uint8_t resetPin) :
	NewSoftSerial(rxPin, txPin) {
	NewSoftSerial::begin(ROBOCLAW_BAUD_RATE);

	m_hasReset = false;
	m_resetPin = resetPin;
	if (resetPin != 100) {
		m_hasReset = true;
		pinMode(m_resetPin, OUTPUT);
		digitalWrite(m_resetPin, HIGH);
	}

	RoboClaw::reset();
	uint8_t k[] = { SENDING_LEFT_ENCODER_SPD_CMD, WAITING_FOR_LEFT_ENCODER_SPD,
			SENDING_RIGHT_ENCODER_SPD_CMD, WAITING_FOR_RIGHT_ENCODER_SPD,
			SENDING_LEFT_ENCODER_CNT_CMD, WAITING_FOR_LEFT_ENCODER_CNT,
			SENDING_RIGHT_ENCODER_CNT_CMD, WAITING_FOR_RIGHT_ENCODER_CNT };
	for (uint8_t i = 0; i < 8; i++)
		m_states[i] = k[i];

	/*	// set PWM resolution
	 byte msg[4];
	 msg[0] = ROBOCLAW_ADDRESS;
	 msg[1] = ROBOCLAW_SET_RESOLUTION;
	 msg[2] = 0x00; // +-255
	 msg[3] = (msg[0] + msg[1] + 0x03) & 0x7f;

	 for (uint8_t i = 0; i < 4; i++) {
	 NewSoftSerial::print(msg[i], BYTE);
	 }

	 this->setPID(ROBOCLAW_PID_P, ROBOCLAW_PID_I, ROBOCLAW_PID_D,
	 ROBOCLAW_MAX_QPPS);*/
}

/**
 * Set PID constants
 */
void RoboClaw::setPID(long p, long i, long d, long qpps) {
	byte msg[19];
	msg[0] = ROBOCLAW_ADDRESS;
	msg[1] = ROBOCLAW_SET_PID_LEFT;

	msg[2] = (byte) (d >> 24);
	msg[3] = (byte) (d >> 16);
	msg[4] = (byte) (d >> 8);
	msg[5] = (byte) (d & 0xFF);

	msg[6] = (byte) (p >> 24);
	msg[7] = (byte) (p >> 16);
	msg[8] = (byte) (p >> 8);
	msg[9] = (byte) (p & 0xFF);

	msg[10] = (byte) (i >> 24);
	msg[11] = (byte) (i >> 16);
	msg[12] = (byte) (i >> 8);
	msg[13] = (byte) (i & 0xFF);

	msg[14] = (byte) (qpps >> 24);
	msg[15] = (byte) (qpps >> 16);
	msg[16] = (byte) (qpps >> 8);
	msg[17] = (byte) (qpps & 0xFF);

	msg[18] = (msg[0] + msg[1] + msg[2] + msg[3] + msg[4] + msg[5] + msg[6]
			+ msg[7] + msg[8] + msg[9] + msg[10] + msg[11] + msg[12] + msg[13]
			+ msg[14] + msg[15] + msg[16] + msg[17]) & 0x7F;

	for (uint8_t iter = 0; iter < 19; iter++) {
		NewSoftSerial::print(msg[iter], BYTE);
	}

	msg[1] = ROBOCLAW_SET_PID_RIGHT;

	msg[18] = (msg[0] + msg[1] + msg[2] + msg[3] + msg[4] + msg[5] + msg[6]
			+ msg[7] + msg[8] + msg[9] + msg[10] + msg[11] + msg[12] + msg[13]
			+ msg[14] + msg[15] + msg[16] + msg[17]) & 0x7F;

	for (uint8_t iter = 0; iter < 19; iter++) {
		NewSoftSerial::print(msg[iter], BYTE);
	}
}

/**
 * Set speeds with duty cycle.
 */
void RoboClaw::setSpeedDutyCycle(int16_t leftDutyCycle, int16_t rightDutyCycle) {
	leftDutyCycle
			= constrain(leftDutyCycle, -ROBOCLAW_RESOLUTION, ROBOCLAW_RESOLUTION);
	rightDutyCycle
			= constrain(rightDutyCycle, -ROBOCLAW_RESOLUTION, ROBOCLAW_RESOLUTION);

	byte msg[10];

	// left packet
	msg[0] = ROBOCLAW_ADDRESS;
	msg[1] = ROBOCLAW_SET_MOTOR_DUTY_CYCLE_LEFT;
	msg[2] = (byte) (leftDutyCycle >> 8); // MSB first
	msg[3] = (byte) (leftDutyCycle & 0xFF);
	msg[4] = (msg[0] + msg[1] + msg[2] + msg[3]) & 0x7f;

	// right packet
	msg[5] = ROBOCLAW_ADDRESS;
	msg[6] = ROBOCLAW_SET_MOTOR_DUTY_CYCLE_RIGHT;
	msg[7] = (byte) (rightDutyCycle >> 8); // MSB first
	msg[8] = (byte) (rightDutyCycle & 0xFF);
	msg[9] = (msg[5] + msg[6] + msg[7] + msg[8]) & 0x7f;

	for (uint8_t i = 0; i < 10; i++) {
		NewSoftSerial::print(msg[i], BYTE);
	}
}

/**
 * Utilize Roboclaw's built in PID routine to drive.
 */
void RoboClaw::setSpeedPID(double leftSpeedCMPS, double rightSpeedCMPS) {
	long leftSpeedQPPS, rightSpeedQPPS;

	// convert cm/s to quadrature encoder counts per second
	leftSpeedQPPS = leftSpeedCMPS * CM_TO_QUAD_COUNTS_PER_SECOND; // signed
	rightSpeedQPPS = rightSpeedCMPS * CM_TO_QUAD_COUNTS_PER_SECOND; // signed

	byte msg[15];
	msg[0] = ROBOCLAW_ADDRESS;
	msg[1] = ROBOCLAW_SET_MOTOR_QPPS_AND_ACCEL;

	long accel = ROBOCLAW_MAX_ACCEL; // constant acceleration here

	// encode acceleration into bytes
	msg[2] = (byte) (accel >> 24); // MSB first
	msg[3] = (byte) (accel >> 16);
	msg[4] = (byte) (accel >> 8);
	msg[5] = (byte) (accel & 0xFF);

	// encoder left speed into bytes
	msg[6] = (byte) (leftSpeedQPPS >> 24); // MSB first
	msg[7] = (byte) (leftSpeedQPPS >> 16);
	msg[8] = (byte) (leftSpeedQPPS >> 8);
	msg[9] = (byte) (leftSpeedQPPS & 0xFF);

	// encode right speed into bytes
	msg[10] = (byte) (rightSpeedQPPS >> 24); // MSB first
	msg[11] = (byte) (rightSpeedQPPS >> 16);
	msg[12] = (byte) (rightSpeedQPPS >> 8);
	msg[13] = (byte) (rightSpeedQPPS & 0xFF);

	// calculate the checksum
	msg[14] = (msg[0] + msg[1] + msg[2] + msg[3] + msg[4] + msg[5] + msg[6]
			+ msg[7] + msg[8] + msg[9] + msg[10] + msg[11] + msg[12] + msg[13])
			& 0x7f;

	// send the packet
	for (uint8_t i = 0; i < 15; i++) {
		NewSoftSerial::print(msg[i], BYTE);
	}
}

void RoboClaw::reset() {
	m_statePtr = 0;
	m_pstatePtr = 0;
	m_lastStateChange = millis();
	m_msgPtr = 0;
	m_msgLen = 0;
	if (m_hasReset) {
		digitalWrite(m_resetPin, LOW);
		delay(1);
		digitalWrite(m_resetPin, HIGH);
	}
}
/**
 * State-based sensor updates
 */
void RoboClaw::updateState(double *leftEncSpeed, double *rightEncSpeed,
		long *leftEncCount, long *rightEncCount) {
	uint8_t bufSize = 24;
	// common code
	switch (m_states[m_statePtr]) {
	case SENDING_LEFT_ENCODER_SPD_CMD:
	case SENDING_RIGHT_ENCODER_SPD_CMD:
	case SENDING_LEFT_ENCODER_CNT_CMD:
	case SENDING_RIGHT_ENCODER_CNT_CMD:
		NewSoftSerial::print(ROBOCLAW_ADDRESS, BYTE);
		break;
	case WAITING_FOR_LEFT_ENCODER_SPD:
	case WAITING_FOR_RIGHT_ENCODER_SPD:
	case WAITING_FOR_LEFT_ENCODER_CNT:
	case WAITING_FOR_RIGHT_ENCODER_CNT:
		while (NewSoftSerial::available()) {
			m_msgBuf[m_msgPtr] = NewSoftSerial::read();
			m_msgPtr = (m_msgPtr + 1) % bufSize;
			m_msgLen++;
		}
		break;
	}
	// specific code
	switch (m_states[m_statePtr]) {
	case SENDING_LEFT_ENCODER_SPD_CMD:
		NewSoftSerial::print(ROBOCLAW_GET_SPEED_LEFT, BYTE);
		RoboClaw::advPtr();
		break;
	case SENDING_RIGHT_ENCODER_SPD_CMD:
		NewSoftSerial::print(ROBOCLAW_GET_SPEED_RIGHT, BYTE);
		RoboClaw::advPtr();
		break;
	case SENDING_LEFT_ENCODER_CNT_CMD:
		NewSoftSerial::print(ROBOCLAW_GET_ENC_LEFT, BYTE);
		RoboClaw::advPtr();
		break;
	case SENDING_RIGHT_ENCODER_CNT_CMD:
		NewSoftSerial::print(ROBOCLAW_GET_ENC_RIGHT, BYTE);
		RoboClaw::advPtr();
		break;
	case WAITING_FOR_LEFT_ENCODER_SPD:
		if (m_msgLen >= 5) {
			long m[5] = { m_msgBuf[(m_msgStart + 0) % bufSize],
					m_msgBuf[(m_msgStart + 1) % bufSize], m_msgBuf[(m_msgStart
							+ 2) % bufSize], m_msgBuf[(m_msgStart + 3)
							% bufSize], m_msgBuf[(m_msgStart + 4) % bufSize] };
#ifdef NEED_HACK
			if (m[0] != 0xFF) {
				for (uint8_t i = 0; i < 5; i++) {
					m[i] &= 0b01111111;
				}
			}
#endif
#ifdef DEBUG
			Serial.print("M1 Encoder: CMD: ");
			Serial.print(ROBOCLAW_GET_SPEED_LEFT, HEX);
			Serial.print(" DATA: ");
			for (uint8_t i = 0; i < 5; i++) {
				Serial.print(m[i], HEX);
				if (i != 4)
				Serial.print(", ");
			}
#endif
			long spd = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
#ifdef DEBUG
			Serial.print(" spd: ");
			Serial.print(spd, DEC);
			Serial.print("\r\n");
#endif
			//			if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_SPEED_LEFT + m[0] + m[1]
			//					+ m[2] + m[3]) & 0x7F) == m[4]) {
			*leftEncSpeed = QUAD_COUNTS_PER_125th_SECOND_TO_CM * spd;
			//			}

			m_msgStart = (m_msgStart + 5) % bufSize;
			m_msgLen -= 5;

			RoboClaw::advPtr();
		}
		break;
	case WAITING_FOR_RIGHT_ENCODER_SPD:
		if (m_msgLen >= 5) {
			long m[5] = { m_msgBuf[(m_msgStart + 0) % bufSize],
					m_msgBuf[(m_msgStart + 1) % bufSize], m_msgBuf[(m_msgStart
							+ 2) % bufSize], m_msgBuf[(m_msgStart + 3)
							% bufSize], m_msgBuf[(m_msgStart + 4) % bufSize] };
#ifdef NEED_HACK
			if (m[0] != 0xFF) {
				for (uint8_t i = 0; i < 5; i++) {
					m[i] &= 0b01111111;
				}
			}
#endif
#ifdef DEBUG
			Serial.print("M2 Encoder: CMD: ");
			Serial.print(ROBOCLAW_GET_SPEED_RIGHT, HEX);
			Serial.print(" DATA: ");
			for (uint8_t i = 0; i < 5; i++) {
				Serial.print(m[i], HEX);
				if (i != 4)
				Serial.print(", ");
			}
#endif
			long spd = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
#ifdef DEBUG
			Serial.print(" spd: ");
			Serial.print(spd, DEC);
			Serial.print("\r\n");
#endif

			//			if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_SPEED_RIGHT + m[0] + m[1]
			//					+ m[2] + m[3]) & 0x7F) == m[4]) {
			*rightEncSpeed = QUAD_COUNTS_PER_125th_SECOND_TO_CM * spd;
			//			}

			m_msgStart = (m_msgStart + 5) % bufSize;
			m_msgLen -= 5;

			RoboClaw::advPtr();
		}
		break;
	case WAITING_FOR_LEFT_ENCODER_CNT:
		if (m_msgLen >= 6) {
			long m[6] = { m_msgBuf[(m_msgStart + 0) % bufSize],
					m_msgBuf[(m_msgStart + 1) % bufSize], m_msgBuf[(m_msgStart
							+ 2) % bufSize], m_msgBuf[(m_msgStart + 3)
							% bufSize], m_msgBuf[(m_msgStart + 4) % bufSize],
					m_msgBuf[(m_msgStart + 5) % bufSize] };
#ifdef NEED_HACK
			if (m[0] != 0xFF) {
				for (uint8_t i = 0; i < 6; i++) {
					m[i] &= 0b01111111;
				}
			}
#endif
			long count = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
			//			if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_ENC_LEFT + m[0] + m[1] + m[2]
			//					+ m[3] + m[4]) & 0x7F) == m[5]) {
			*leftEncCount = count;
			//			}
			m_msgStart = (m_msgStart + 6) % bufSize;
			m_msgLen -= 6;

			RoboClaw::advPtr();
		}
		break;
	case WAITING_FOR_RIGHT_ENCODER_CNT:
		if (m_msgLen >= 6) {
			long m[6] = { m_msgBuf[(m_msgStart + 0) % bufSize],
					m_msgBuf[(m_msgStart + 1) % bufSize], m_msgBuf[(m_msgStart
							+ 2) % bufSize], m_msgBuf[(m_msgStart + 3)
							% bufSize], m_msgBuf[(m_msgStart + 4) % bufSize],
					m_msgBuf[(m_msgStart + 5) % bufSize] };
#ifdef NEED_HACK
			if (m[0] != 0xFF) {
				for (uint8_t i = 0; i < 6; i++) {
					m[i] &= 0b01111111;
				}
			}
#endif
			long count = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
			//			if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_ENC_LEFT + m[0] + m[1] + m[2]
			//					+ m[3] + m[4]) & 0x7F) == m[5]) {
			*rightEncCount = count;
			//			}

			m_msgStart = (m_msgStart + 6) % bufSize;
			m_msgLen -= 6;

			RoboClaw::advPtr();
		}
		break;
	}

	if (millis() - m_lastStateChange > STATE_TIMEOUT && (m_pstatePtr
			== m_statePtr)) {
		RoboClaw::advPtr();
	}
}

void RoboClaw::advPtr() {
	m_lastStateChange = millis();
	m_pstatePtr = m_statePtr;
	m_statePtr = (m_statePtr + 1) % 8;
}

/**
 * Helper routine to get current encoder speeds
 */
void RoboClaw::updateSpeed(double *leftEncSpeed, double *rightEncSpeed) {
	RoboClaw::getEncoderSpeedLeft(leftEncSpeed);
	RoboClaw::getEncoderSpeedRight(rightEncSpeed);
}

/**
 * Get encoder speed of left motor
 */
void RoboClaw::getEncoderSpeedLeft(double *speed) {
	NewSoftSerial::print(ROBOCLAW_ADDRESS, BYTE);
	NewSoftSerial::print(ROBOCLAW_GET_SPEED_LEFT, BYTE);

	long m[4] = { 0, 0, 0, 0 };
	for (uint8_t i = 0; i < 4; i++) {
		m[i] = RoboClaw::nextByte(100);
	}
	uint8_t checksum = RoboClaw::nextByte(100);
#ifdef NEED_HACK
	if (m[0] != 0xFF) {
		for (uint8_t i = 0; i < 4; i++) {
			m[i] &= 0b01111111;
		}
	}
#endif

	long spd = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
	if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_SPEED_LEFT + m[0] + m[1] + m[2]
			+ m[3]) & 0x7F) == checksum)
		*speed = /*QUAD_COUNTS_PER_125th_SECOND_TO_CM * */spd;
}

/**
 * Get current encoder speed of right motor.
 */
void RoboClaw::getEncoderSpeedRight(double *speed) {
	NewSoftSerial::print(ROBOCLAW_ADDRESS, BYTE);
	NewSoftSerial::print(ROBOCLAW_GET_SPEED_RIGHT, BYTE);

	long m[4] = { 0, 0, 0, 0 };
	for (uint8_t i = 0; i < 4; i++) {
		m[i] = RoboClaw::nextByte(100);
	}
	uint8_t checksum = RoboClaw::nextByte(100);
#ifdef NEED_HACK
	if (m[0] != 0xFF) {
		for (uint8_t i = 0; i < 4; i++) {
			m[i] &= 0b01111111;
		}
	}
#endif

	long spd = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
	if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_SPEED_RIGHT + m[0] + m[1] + m[2]
			+ m[3]) & 0x7F) == checksum)
		*speed = /*QUAD_COUNTS_PER_125th_SECOND_TO_CM * */spd;
}

/**
 * Helper routine to get the current encoder counts.
 */
void RoboClaw::updateEnc(long *leftEncCount, long *rightEncCount) {
	getEncLeft(leftEncCount);
	getEncRight(rightEncCount);
}

/**
 * Get left encoder counts.
 */
void RoboClaw::getEncLeft(long *enc) {
	NewSoftSerial::print(ROBOCLAW_ADDRESS, BYTE);
	NewSoftSerial::print(ROBOCLAW_GET_ENC_LEFT, BYTE);
	long m[4];
	for (uint8_t i = 0; i < 4; i++) {
		m[i] = RoboClaw::nextByte(100);
	}
	uint8_t status = RoboClaw::nextByte(100);
	uint8_t checksum = RoboClaw::nextByte(100);
#ifdef NEED_HACK
	if (m[0] != 0xFF) {
		for (uint8_t i = 0; i < 4; i++) {
			m[i] &= 0b01111111;
		}
	}
#endif

	long count = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
	if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_ENC_LEFT + status + m[0] + m[1]
			+ m[2] + m[3]) & 0x7F) == checksum)
		*enc = count;
}

/**
 * Get right encoder counts
 */
void RoboClaw::getEncRight(long *enc) {
	NewSoftSerial::print(ROBOCLAW_ADDRESS, BYTE);
	NewSoftSerial::print(ROBOCLAW_GET_ENC_RIGHT, BYTE);
	long m[4];
	for (uint8_t i = 0; i < 4; i++) {
		m[i] = RoboClaw::nextByte(100);
	}
	uint8_t status = RoboClaw::nextByte(100);
	uint8_t checksum = RoboClaw::nextByte(100);
#ifdef NEED_HACK
	if (m[0] != 0xFF) {
		for (uint8_t i = 0; i < 4; i++) {
			m[i] &= 0b01111111;
		}
	}
#endif

	long count = m[0] << 24 | m[1] << 16 | m[2] << 8 | m[3];
	if (((ROBOCLAW_ADDRESS + ROBOCLAW_GET_ENC_RIGHT + status + m[0] + m[1]
			+ m[2] + m[3]) & 0x7F) == checksum)
		*enc = count;
}

/**
 * Get motor battery voltage from RoboClaw
 */
double RoboClaw::getMotorBatteryVoltage() {
	NewSoftSerial::print(ROBOCLAW_ADDRESS, BYTE);
	NewSoftSerial::print(ROBOCLAW_GET_MAIN_BATTERY_VOLTAGE, BYTE);
	int m[2] = { 0, 0 };
	for (uint8_t i = 0; i < 2; i++) {
		m[i] = RoboClaw::nextByte(100);
	}
	RoboClaw::nextByte(100);
	double voltage = (m[0] << 8 | m[1]) * 0.1;
	return voltage;
}

/**
 * Helper routine to get the next byte from USART
 */
char RoboClaw::nextByte(unsigned long timeout) {
	char recv = 0x00;
	unsigned long timeOutTime = millis() + timeout;
	while (millis() < timeOutTime && !NewSoftSerial::available()) {
		// do nothing
	}
	if (NewSoftSerial::available()) {
		recv = NewSoftSerial::read();
	}
	return recv;
}

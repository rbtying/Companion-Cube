/*
 * RoboClaw.h
 *
 *  Created on: Feb 20, 2011
 *      Author: rbtying
 */

#ifndef ROBOCLAW_H_
#define ROBOCLAW_H_

#include "NewSoftSerial.h"

// RoboClaw commands
// general
#define ROBOCLAW_ADDRESS 0x80
#define ROBOCLAW_SET_RESOLUTION 0x30
#define ROBOCLAW_SET_PID_LEFT 0x1C
#define ROBOCLAW_SET_PID_RIGHT 0x1D
#define ROBOCLAW_RESOLUTION 0x7F
#define ROBOCLAW_GET_MAIN_BATTERY_VOLTAGE 0x18
// encoder speed
#define ROBOCLAW_GET_SPEED_LEFT 0x1E // M1
#define ROBOCLAW_GET_SPEED_RIGHT 0x1F // M2
// encoder count
#define ROBOCLAW_GET_ENC_LEFT 0x10 // M1
#define ROBOCLAW_GET_ENC_RIGHT 0x11 // M2
// set speeds
#define ROBOCLAW_SET_MOTOR_DUTY_CYCLE_LEFT 0x20 // M1
#define ROBOCLAW_SET_MOTOR_DUTY_CYCLE_RIGHT 0x21 // M2
#define ROBOCLAW_SET_MOTOR_QPPS 0x25
#define ROBOCLAW_SET_MOTOR_QPPS_AND_ACCEL 0x28

// conversion factors
#define QUAD_COUNTS_PER_125th_SECOND_TO_CM 0.122266859 // mathematically derived
#define CM_TO_QUAD_COUNTS_PER_SECOND 3211.8195 // mathematically derived
#define CM_TO_QUAD_COUNTS_PER_SECOND_LEFT 3211.8195
#define CM_TO_QUAD_COUNTS_PER_SECOND_RIGHT 3211.8195

// constants
#define ROBOCLAW_MAX_ACCEL 10000 // in QPPS
#define ROBOCLAW_BAUD_RATE 19200
#define ROBOCLAW_MAX_QPPS 240000
#define ROBOCLAW_PID_P 0x00000200
#define ROBOCLAW_PID_I 0x00000100
#define ROBOCLAW_PID_D 0x00000080

// states
#define STATE_TIMEOUT 100
#define WAITING_FOR_LEFT_ENCODER_SPD 0x10
#define WAITING_FOR_RIGHT_ENCODER_SPD 0x11
#define WAITING_FOR_LEFT_ENCODER_CNT 0x12
#define WAITING_FOR_RIGHT_ENCODER_CNT 0x13
#define SENDING_LEFT_ENCODER_SPD_CMD 0x20
#define SENDING_RIGHT_ENCODER_SPD_CMD 0x21
#define SENDING_LEFT_ENCODER_CNT_CMD 0x22
#define SENDING_RIGHT_ENCODER_CNT_CMD 0x23

class RoboClaw: NewSoftSerial {
public:
	RoboClaw(uint8_t rxPin, uint8_t txPin, uint8_t resetPin = 100);
	void setSpeedDutyCycle(int16_t leftSpeed, int16_t rightSpeed);
	void setSpeedPID(double leftSpeed, double rightSpeed);
	void setPID(long p = ROBOCLAW_PID_P, long i = ROBOCLAW_PID_I, long d =
			ROBOCLAW_PID_D, long qpps = ROBOCLAW_MAX_QPPS);
	void updateSpeed(double *leftEncSpeed, double *rightEncSpeed);
	void updateEnc(long *leftEncCount, long *rightEncCount);
	double getMotorBatteryVoltage();
	void updateState(double *leftEncSpeed, double *rightEncSpeed,
			long *leftEncCount, long *rightEncCount);
	void reset();

private:
	bool m_hasReset;
	uint8_t m_resetPin;
	uint8_t m_statePtr, m_pstatePtr;
	uint8_t m_msgPtr, m_msgStart, m_msgLen, m_msgBuf[24];
	uint8_t m_states[8];
	unsigned long m_lastStateChange;
	void advPtr();
	void getEncoderSpeedLeft(double *speed);
	void getEncoderSpeedRight(double *speed);
	void getEncLeft(long *enc);
	void getEncRight(long *enc);
	char nextByte(unsigned long timeout = 100);
};
#endif /* ROBOCLAW_H_ */

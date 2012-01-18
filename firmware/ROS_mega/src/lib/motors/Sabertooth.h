/*
 * Sabertooth.h
 *
 *  Created on: Aug 31, 2011
 *      Author: Robert Ying
 *
 *  Software serial transmission taken from NewSoftSerial
 */

#ifndef SABERTOOTH_H_
#define SABERTOOTH_H_

#include <WProgram.h>
#include <HardwareSerial.h>
#include "DualMotor.h"

#define SBT_BAUDING_CHAR 0xaa

#define SBT_OP_M1_FWD 0
#define SBT_OP_M1_REV 1
#define SBT_OP_MIN_VOLT 2
#define SBT_OP_MAX_VOLT 3
#define SBT_OP_M2_FWD 4
#define SBT_OP_M2_REV 5

class Sabertooth: DualMotor {
public:
	Sabertooth(uint8_t address, uint8_t txPin);
	Sabertooth(uint8_t address, HardwareSerial * hws);

	void disable();
	void enable();
	bool enabled();

	void setMinVoltage(float voltage);
	void setMaxVoltage(float voltage);

	void setSpeed(int8_t left, int8_t right);
	void setSpeed(int8_t left, int8_t right, bool leftFwd, bool rightFwd);
	void setSpeed(motor_data * data);
	void setM1Speed(int8_t spd);
	void setM2Speed(int8_t spd);
	int8_t getLeftSpeed();
	int8_t getRightSpeed();

private:
	bool m_usinghws;
	bool m_enabled;
	HardwareSerial * m_hws;
	uint8_t m_address;
	int16_t m_left_speed, m_right_speed;

	uint8_t m_transmitBitMask;
	volatile uint8_t *m_transmitPortRegister;

	virtual void write(uint8_t byte);
	static inline void tunedDelay(uint16_t delay);

	void sendPacket(uint8_t op, uint8_t data);
	uint8_t checksum(uint8_t op, uint8_t data);
	void setMotorSpeed(uint8_t op, uint8_t spd);
};

#endif /* SABERTOOTH_H_ */

/*
 * Sabertooth.cc
 *
 *  Created on: Aug 31, 2011
 *      Author: Robert Ying
 *
 *  Software serial transmission taken from NewSoftSerial
 */

#include "Sabertooth.h"
#include "WConstants.h"
#include "pins_arduino.h"

// 19200 baud
#if F_CPU == 16000000
const int TX_DELAY = 114;
const int XMIT_START_ADJUSTMENT = 5;
#elif F_CPU == 8000000
const int TX_DELAY = 52;
const int XMIT_START_ADJUSTMENT = 4;
#elif F_CPU == 20000000
const int TX_DELAY = 145;
const int XMIT_START_ADJUSTMENT = 6;
#else
#error This version of the Sabertooth library only supports 8Mhz/16Mhz/20Mhz processors
#endif

Sabertooth::Sabertooth(uint8_t address, uint8_t txPin) {

	m_address = address;

	pinMode(txPin, OUTPUT);
	digitalWrite(txPin, HIGH);
	m_transmitBitMask = digitalPinToBitMask(txPin);
	uint8_t port = digitalPinToPort(txPin);
	m_transmitPortRegister = portOutputRegister(port);
	tunedDelay(TX_DELAY);

	reset();
}

inline void Sabertooth::tunedDelay(uint16_t delay) {
	uint8_t tmp = 0;

	asm volatile("sbiw    %0, 0x01 \n\t"
			"ldi %1, 0xFF \n\t"
			"cpi %A0, 0xFF \n\t"
			"cpc %B0, %1 \n\t"
			"brne .-10 \n\t"
			: "+w" (delay), "+a" (tmp)
			: "0" (delay)
	);
}

void Sabertooth::reset() {
	write(SBT_BAUDING_CHAR);
}

void Sabertooth::setMinVoltage(float voltage) {
	voltage = max(voltage, 6.0);
	uint8_t data = (uint8_t) ((voltage - 6.0) * 5.0);
	sendPacket(SBT_OP_MIN_VOLT, data);
}

void Sabertooth::setMaxVoltage(float voltage) {
	uint8_t data = (uint8_t) (voltage * 5.12);
	sendPacket(SBT_OP_MAX_VOLT, data);
}

void Sabertooth::setSpeed(int8_t left, int8_t right) {
	reset();
	setLeftSpeed(left);
	setRightSpeed(right);
}

void Sabertooth::setSpeed(int8_t left, int8_t right, bool leftFwd,
		bool rightFwd) {
	setSpeed((leftFwd ? (1) : (-1)) * left, (rightFwd ? (1) : (-1)) * right);
}

int8_t Sabertooth::getLeftSpeed() {
	return m_left_speed;
}

int8_t Sabertooth::getRightSpeed() {
	return m_right_speed;
}

void Sabertooth::setLeftSpeed(int8_t spd) {
	m_left_speed = spd;
	if (spd >= 0)
		setMotorSpeed(SBT_OP_M2_FWD, abs(spd));
	else
		setMotorSpeed(SBT_OP_M2_REV, abs(spd));
}

void Sabertooth::setRightSpeed(int8_t spd) {
	m_right_speed = spd;
	if (spd >= 0)
		setMotorSpeed(SBT_OP_M1_FWD, abs(spd));
	else
		setMotorSpeed(SBT_OP_M1_REV, abs(spd));
}

void Sabertooth::setMotorSpeed(uint8_t op, uint8_t spd) {
	spd = constrain(spd, 0, 127);
	sendPacket(op, spd);
}

void Sabertooth::sendPacket(uint8_t op, uint8_t data) {
	write(m_address);
	write(op);
	write(data);
	write(checksum(op, data));
}

uint8_t Sabertooth::checksum(uint8_t op, uint8_t data) {
	return (m_address + op + data) & 0b01111111;
}

void Sabertooth::write(uint8_t b) {
	uint8_t oldSREG = SREG;
	cli();
	*m_transmitPortRegister &= ~m_transmitBitMask;
	tunedDelay(TX_DELAY + XMIT_START_ADJUSTMENT);

	for (byte mask = 0x01; mask; mask <<= 1) {
		if (b & mask) // choose bit
			*m_transmitPortRegister |= m_transmitBitMask; // send 1
		else
			*m_transmitPortRegister &= ~m_transmitBitMask; // send 0
		tunedDelay(TX_DELAY);
	}

	*m_transmitPortRegister |= m_transmitBitMask;

	SREG = oldSREG; // turn interrupts back on
	tunedDelay(TX_DELAY);
}

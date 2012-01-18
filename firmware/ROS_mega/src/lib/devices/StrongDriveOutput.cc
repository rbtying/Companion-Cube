/*
 * StrongDriveOutput.cc
 *
 *  Created on: Dec 10, 2011
 *      Author: rbtying
 */

#include "StrongDriveOutput.h"
#include "../utilities/fastIO.h"

StrongDriveOutput::StrongDriveOutput(uint8_t pin, bool inverted) {
	m_inverted = inverted;
	m_port = digitalPinToPortReg(pin);
	m_pin = digitalPinToBit(pin);
	fastIOMode(pin, OUTPUT);
}

void StrongDriveOutput::on() {
	if (m_inverted) {
		*m_port &= ~(1 << m_pin);
	} else {
		*m_port |= 1 << m_pin;
	}
}

void StrongDriveOutput::off() {
	if (m_inverted) {
		*m_port |= 1 << m_pin;
	} else {
		*m_port &= ~(1 << m_pin);
	}
}

void StrongDriveOutput::set(bool b) {
	if (b) {
		on();
	} else {
		off();
	}
}
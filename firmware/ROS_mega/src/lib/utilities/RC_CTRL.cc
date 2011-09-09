/*
 * RC_CTRL.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: rbtying
 */

#include "RC_CTRL.h"

/**
 * Constructs an RC controller
 */
RC_CTRL::RC_CTRL(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5) {
	m_pins[0] = c1;
	m_pins[1] = c2;
	m_pins[2] = c3;
	m_pins[3] = c4;
	m_pins[4] = c5;
	for (uint8_t i = 0; i < 5; i++) {
		pinMode(m_pins[i], INPUT);
		m_expo[i] = 0;
	}
	RC_CTRL::update();
}

/**
 * Returns a 8-bit value showing the position of the channel (channels 1-5)
 */
uint8_t RC_CTRL::getChannel(uint8_t channel) {
	if (channel > 0 && channel < 6)
		return m_channels[channel - 1];
	else
		return 127;
}

int8_t RC_CTRL::deadBandProcessor(int8_t val, uint8_t deadband) {
	val = constrain(val, -127, 127);
	if (abs(val) > deadband) {
		uint8_t newMinMax = 127 - deadband;
		int8_t sign;
		if (val < 0)
			sign = -1;
		else
			sign = 1;
		val = sign * (abs(val) - deadband);
		val = map(val, -(newMinMax), (newMinMax), -127, 127);
	} else {
		val = 0;
	}
	return val;
}

/**
 * Returns the y-axis reading of either the right or the left stick
 */
int8_t RC_CTRL::getThrottle(bool left, uint8_t deadband) {
	int8_t throttle;
	if (left)
		throttle = m_channels[0] - 127;
	else
		throttle = m_channels[2] - 127;
	return RC_CTRL::deadBandProcessor(throttle, deadband);
}

/**
 * Returns the x-axis reading of either the right or the left stick
 */
int8_t RC_CTRL::getYaw(bool left, uint8_t deadband) {
	int8_t yaw;
	if (left)
		yaw = m_channels[3] - 127;
	else
		yaw = m_channels[1] - 127;
	return RC_CTRL::deadBandProcessor(yaw, deadband);
}

/**
 * Returns the channel-5 reading
 */
uint8_t RC_CTRL::getKnob() {
	return m_channels[4];
}

/**
 * Updates all channels
 */
void RC_CTRL::update() {
	m_channels[0] = readRC(0);
	m_channels[2] = readRC(2);
	m_channels[4] = readRC(4);
	m_channels[1] = readRC(1);
	m_channels[3] = readRC(3);
}

/**
 * returns the value of a single channel
 */
uint8_t RC_CTRL::readRC(uint8_t channel) {
	uint16_t pulse = pulseIn(m_pins[channel], HIGH);
	if (pulse < 750 || pulse > 2250) {
		return 127;
	} else {
		return map(constrain(pulse, 1000, 2000), 1000, 2000, 0, 255);
	}
}

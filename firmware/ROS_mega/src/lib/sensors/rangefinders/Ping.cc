/*
 * Ping.cpp
 *
 *  Created on: Feb 12, 2011
 *      Author: rbtying
 */

#include "Ping.h"
#include <Arduino.h>

/**
 * Constructs a Ping object
 */
Ping::Ping(uint8_t pin) {
	m_pin = pin;
}

/**
 * Gets the distance measured in CM
 */
double Ping::get_dist() {
	pinMode(m_pin, OUTPUT);
	digitalWrite(m_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(m_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(m_pin, LOW);

	pinMode(m_pin, INPUT);
	uint16_t duration = pulseIn(m_pin, HIGH);

	return duration / 58;
}

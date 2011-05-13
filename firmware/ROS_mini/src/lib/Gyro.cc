/*
 * Gyro.cpp
 *
 *  Created on: Apr 18, 2011
 *      Author: rbtying
 */

#include "Gyro.h"

Gyro::Gyro(uint8_t axisPin, uint8_t refPin, float conversion_factor) {
	m_axisPin = axisPin;
	m_refPin = refPin;
	m_conversion_factor = conversion_factor;

	m_usingMux = false;

	pinMode(m_axisPin, INPUT);
	pinMode(m_refPin, INPUT);
	digitalWrite(m_axisPin, LOW);
	digitalWrite(m_refPin, LOW);
}

Gyro::Gyro(CD74HC4067 *mux, uint8_t axisPin, uint8_t refPin,
		float conversion_factor = LPR510_CONVERSION_FACTOR) {
	m_axisPin = axisPin;
	m_refPin = refPin;
	m_conversion_factor = conversion_factor;
	m_usingMux = true;
	m_mux = mux;
}

void Gyro::calibrate(uint16_t num, bool continuous) {
	int adcAxis, adcRef;
	if (!continuous) {
		m_offset = 0;
		for (uint16_t i = 0; i < num; i++) {
			if (m_usingMux) {
				adcAxis = m_mux->readADC(m_axisPin);
				adcRef = m_mux->readADC(m_refPin);
			} else {
				adcAxis = analogRead(m_axisPin);
				adcRef = analogRead(m_refPin);
			}
			m_offset += (adcAxis - adcRef) * m_conversion_factor;
			delay(1);
		}
		m_offset /= num * 1.0;
	} else {
		m_offset *= (num - 1) / (num * 1.0);
		if (m_usingMux) {
			adcAxis = m_mux->readADC(m_axisPin);
			adcRef = m_mux->readADC(m_refPin);
		} else {
			adcAxis = analogRead(m_axisPin);
			adcRef = analogRead(m_refPin);
		}
		m_offset += (adcAxis - adcRef) * m_conversion_factor / (num * 1.0);
	}
}

float Gyro::getValue() {
	int adcAxis, adcRef;
	if (m_usingMux) {
		adcAxis = m_mux->readADC(m_axisPin);
		adcRef = m_mux->readADC(m_refPin);
	} else {
		adcAxis = analogRead(m_axisPin);
		adcRef = analogRead(m_refPin);
	}
	float reading = (adcAxis - adcRef) * m_conversion_factor - m_offset;
	return reading;
}

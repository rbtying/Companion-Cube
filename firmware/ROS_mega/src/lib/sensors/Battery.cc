/*
 * PowerMonitor.cpp
 *
 *  Created on: Apr 11, 2011
 *      Author: rbtying
 */

#include "Battery.h"

Battery::Battery() {
	Battery::Battery(0, 0);
}

Battery::Battery(uint8_t vPin, uint8_t iPin) {
	Battery::set(NULL, vPin, iPin);
}

Battery::Battery(CD74HC4067 *mux, uint8_t vPin, uint8_t iPin) {
	Battery::set(mux, vPin, iPin);
}

void Battery::set(CD74HC4067 *mux, uint8_t vPin, uint8_t iPin) {
	if (mux == NULL) {
		m_usingMux = false;
	} else {
		m_usingMux = true;
		m_mux = mux;
	}

	if (vPin != 0 && iPin != 0) {
		m_vPin = vPin;
		m_iPin = iPin;

		if (!m_usingMux) {
			pinMode(m_vPin, INPUT);
			pinMode(m_iPin, INPUT);
			digitalWrite(m_vPin, LOW); // turn off pullup
			digitalWrite(m_iPin, LOW); // turn off pullup
		}
	}
}

float Battery::getVoltage() {
	uint16_t adcRead;
	if (m_usingMux) {
		adcRead = m_mux->readADC(m_vPin);
	} else {
		adcRead = analogRead(m_vPin);
	}
	float voltage = adcRead * 0.076660156; // 1v:15.7v voltage divider
	return voltage;
}

float Battery::getCurrent() {
	uint16_t adcRead;
	if (m_usingMux) {
		adcRead = m_mux->readADC(m_iPin);
	} else {
		adcRead = analogRead(m_iPin);
	}
	float current = adcRead * 0.13351995; // 90.15A:3.3v scaling factor
	return current;
}

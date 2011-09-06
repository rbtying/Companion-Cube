/*
 * PowerMonitor.cpp
 *
 *  Created on: Apr 11, 2011
 *      Author: rbtying
 */

#include "WProgram.h"
#include "PowerMonitor.h"

PowerMonitor::PowerMonitor() {
	PowerMonitor::PowerMonitor(0, 0);
}

PowerMonitor::PowerMonitor(uint8_t vPin, uint8_t iPin) {
	PowerMonitor::set(NULL, vPin, iPin);
}

PowerMonitor::PowerMonitor(CD74HC4067 *mux, uint8_t vPin, uint8_t iPin) {
	PowerMonitor::set(mux, vPin, iPin);
}

void PowerMonitor::set(CD74HC4067 *mux, uint8_t vPin, uint8_t iPin) {
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

float PowerMonitor::getVoltage() {
	uint16_t adcRead;
	if (m_usingMux) {
		adcRead = m_mux->readADC(m_vPin);
	} else {
		adcRead = analogRead(m_vPin);
	}
	float voltage = adcRead * 0.076660156; // 1v:15.7v voltage divider
	return voltage;
}

float PowerMonitor::getCurrent() {
	uint16_t adcRead;
	if (m_usingMux) {
		adcRead = m_mux->readADC(m_iPin);
	} else {
		adcRead = analogRead(m_iPin);
	}
	float current = adcRead * 0.13351995; // 90.15A:3.3v scaling factor
	return current;
}

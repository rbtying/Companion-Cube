/*
 * PowerMonitor.h
 *
 *  Created on: Apr 11, 2011
 *      Author: rbtying
 */

#ifndef POWERMONITOR_H_
#define POWERMONITOR_H_

#include <WProgram.h>
#include "devices/CD74HC4067.h"

class Battery {
public:
	Battery();
	Battery(uint8_t vPin, uint8_t iPin);
	Battery(CD74HC4067 *mux, uint8_t vPin, uint8_t iPin);
	void set(CD74HC4067 *mux, uint8_t vPin, uint8_t iPin);
	float getVoltage();
	float getCurrent();
private:
	bool m_usingMux;
	CD74HC4067 *m_mux;
	uint8_t m_vPin, m_iPin;
};

#endif /* POWERMONITOR_H_ */

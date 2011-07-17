/*
 * PowerMonitor.h
 *
 *  Created on: Apr 11, 2011
 *      Author: rbtying
 */

#ifndef POWERMONITOR_H_
#define POWERMONITOR_H_

class PowerMonitor {
public:
	PowerMonitor();
	PowerMonitor(uint8_t vPin, uint8_t iPin);
	void set(uint8_t vPin, uint8_t iPin);
	float getVoltage();
	float getCurrent();
private:
	uint8_t m_vPin, m_iPin;
};

#endif /* POWERMONITOR_H_ */

/*
 * CD74HC4067.h
 *
 *  Created on: Apr 28, 2011
 *      Author: rbtying
 */

#include <Arduino.h>

#ifndef CD74HC4067_H_
#define CD74HC4067_H_

class CD74HC4067 {
public:
	CD74HC4067(uint8_t mux1, uint8_t mux2, uint8_t mux3, uint8_t mux4,
			uint8_t mux_adc);
	uint16_t readADC(uint8_t num);
	bool readDigital(uint8_t num);
private:
	uint8_t m_pins[4], m_adc, m_pnum;
	void setMux(uint8_t num);
};

#endif /* CD74HC4067_H_ */

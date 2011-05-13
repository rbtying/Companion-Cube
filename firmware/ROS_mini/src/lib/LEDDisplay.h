/*
 * LEDDisplay.h
 *
 *  Created on: Feb 10, 2011
 *      Author: rbtying
 */

#ifndef LEDDISPLAY_H_
#define LEDDISPLAY_H_

#include "WProgram.h"
#include "NewSoftSerial.h"

#define LEDDISPLAY_BAUD_RATE 9600

class LEDDisplay: NewSoftSerial {
public:
	LEDDisplay(uint8_t txPin);
	void display(int val, char padChar = 'x');
	void setDots(bool a, bool b, bool c, bool d, bool e, bool f);
	void clear();
private:
	uint8_t m_pin;
	int findNumDigits(int val, int base = 10);
};

#endif /* LEDDISPLAY_H_ */

/*
 * LEDDisplay.cpp
 *
 *  Created on: Feb 10, 2011
 *      Author: rbtying
 */

#include "LEDDisplay.h"
#include "NewSoftSerial.h"
#include "WProgram.h"

/**
 * Construct an LEDDisplay
 */
LEDDisplay::LEDDisplay(uint8_t txPin) :
	NewSoftSerial(txPin, txPin) {
	NewSoftSerial::begin(LEDDISPLAY_BAUD_RATE);
	pinMode(txPin, OUTPUT);
}

/**
 * Set the various dots on the display
 */
void LEDDisplay::setDots(bool a, bool b, bool c, bool d, bool e, bool f) {
	NewSoftSerial::print("w");
	uint8_t out = a | b << 1 | c << 2 | d << 3 | e << 4 | f << 5;
	NewSoftSerial::print(out, BYTE);
}

void LEDDisplay::display(int val, char padChar) {
	NewSoftSerial::print("v");
	int numDigits = findNumDigits(val, 10);

	if (val == 0) {
		NewSoftSerial::print("v");
		NewSoftSerial::print(padChar);
		NewSoftSerial::print(padChar);
		NewSoftSerial::print(padChar);
		NewSoftSerial::print("0");
		return;
	}

	if ((val >= 0 && numDigits > 4) || (val < 0 && numDigits > 3)) {
		NewSoftSerial::print("v");
		NewSoftSerial::print("-0F-");
		return;
	}

	int digitsLeft = 4 - numDigits;
	while (digitsLeft > 0) {
		if (digitsLeft != 1 || val >= 0)
			NewSoftSerial::print(padChar);
		else if (val < 0) {
			NewSoftSerial::print("-");
		}
		--digitsLeft;
	}

	int digits[4];
	val = abs(val);
	for (int i = 0; i < 4; i++) {
		digits[i] = val % 10;
		val /= 10;
	}
	for (int i = numDigits - 1; i >= 0; i--) {
		NewSoftSerial::print(digits[i]);
	}
}

/**
 * Find the number of digits in a value given a base
 */
int LEDDisplay::findNumDigits(int val, int base) {
	if (val < 0)
		return findNumDigits(abs(val), base);
	if (val != 0)
		return 1 + findNumDigits(val / base, base);
	return 0;
}

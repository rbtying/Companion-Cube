/*
 * TSL1401.cpp
 *
 *  Created on: Feb 12, 2011
 *      Author: rbtying
 */

#include "TSL1401.h"
#include "Arduino.h"

/**
 * Constructs a new TSL1401
 */
TSL1401::TSL1401(uint8_t adcIn, uint8_t clockOut, uint8_t siOut,
		uint16_t exposure) {
	// store parameters in member vars
	m_adcIn = adcIn;
	m_clockOut = clockOut;
	m_siOut = siOut;
	m_exposure = exposure;

	pinMode(m_siOut, OUTPUT);
	pinMode(m_clockOut, OUTPUT);

	// set up the array
	TSL1401::flush();
}

/**
 * Updates the array
 */
void TSL1401::updateReadings() {
	TSL1401::resetSensor(); // reset the sensor
	delayMicroseconds(m_exposure); // expose the sensor

	// start another frame
	TSL1401::prepFrame();
	// iterate through all the pixels
	for (uint8_t i = 0; i < NUM_PIXELS; i++) {
		// read the pixel data in
		m_pixelDataArr[i] = analogRead(m_adcIn);

		// clock
		digitalWrite(m_clockOut, LOW);
		delayMicroseconds(1);
		digitalWrite(m_clockOut, HIGH);
	}
}

/**
 * Returns a pointer to the array
 */
uint16_t * TSL1401::getDataPointer() {
	return m_pixelDataArr;
}

/**
 * Returns the size of the array
 */
uint8_t TSL1401::getDataSize() {
	return NUM_PIXELS;
}

/**
 * Changes the exposure
 */
void TSL1401::setExpo(uint16_t exposure) {
	m_exposure = exposure;
}

void TSL1401::flush() {
	for (uint8_t i = 0; i < NUM_PIXELS; i++) {
		m_pixelDataArr[i] = 0;
	}
}

TSL1401::~TSL1401() {
	free(m_pixelDataArr);
}

/**
 * Prepares the sensor for a new frame
 */
void TSL1401::prepFrame() {
	digitalWrite(m_clockOut, LOW); // set clock low
	delayMicroseconds(1); // wait for sensor to read the clock
	digitalWrite(m_siOut, HIGH); // start a new frame
	digitalWrite(m_clockOut, HIGH); // toggle clock
	digitalWrite(m_siOut, LOW); // reset frame
}
/**
 * Resets the sensor to set exposure time
 */
void TSL1401::resetSensor() {
	TSL1401::prepFrame();
	// iterate through all pixels
	for (uint8_t i = 0; i < NUM_PIXELS; i++) {
		// just clock, don't bother reading (all data is to be discarded b/c exposure unknown)
		digitalWrite(m_clockOut, LOW);
		delayMicroseconds(1);
		digitalWrite(m_clockOut, HIGH);
	}
}

/**
 * Finds the line
 * 0 is center
 * returns value from -512 to 512 (arbitrary)
 */
int16_t TSL1401::getLinePos(uint16_t noise_threshold, uint16_t line_threshold) {
	uint32_t sum = 0, avg = 0;
	uint16_t value;
	bool onLine = false;
	for (uint8_t i = 0; i < NUM_PIXELS; i++) {
		value = max(m_pixelDataArr[i] - noise_threshold, 0); // remove a standard noise threshold

		if (value > 0) {
			onLine = onLine | (value > line_threshold); // if the line has been seen
			avg += value * i * 10; // weight position by thresholded value, better would be to weight based on values around this one as well (exponential decrease)
			sum += value; // add to total sum
		}
	}

	if (!onLine) {
		if (m_lastLinePos < 0) {
			return -512;
		} else {
			return 512;
		}
	} else {
		avg = avg / sum; // find the average in range [0, NUM_PIXELS * 10]
		m_lastLinePos = map(avg, 0, NUM_PIXELS * 10, -512, 512); // map to requested range
		return m_lastLinePos;
	}

}

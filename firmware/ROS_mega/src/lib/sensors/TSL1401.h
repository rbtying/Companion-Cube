/*
 * TSL1401.h
 *
 *  Created on: Feb 12, 2011
 *      Author: rbtying
 */

#ifndef TSL1401_H_
#define TSL1401_H_
#include <WProgram.h>

#define NUM_PIXELS 128
#define DEFAULT_EXPO (10)

class TSL1401 {
public:
	TSL1401(uint8_t adcIn, uint8_t clockOut, uint8_t siOut, uint16_t exposure =
			DEFAULT_EXPO);
	void updateReadings();
	uint16_t * getDataPointer();
	uint8_t getDataSize();
	int16_t getLinePos(uint16_t noise_threshold, uint16_t line_threshold);
	void setExpo(uint16_t exposure);
	void flush();
	~TSL1401();

private:
	uint8_t m_adcIn, m_clockOut, m_siOut; // pins
	int16_t m_lastLinePos;
	uint16_t m_pixelDataArr[NUM_PIXELS]; // data
	uint16_t m_exposure; // exposure time
	void resetSensor();
	void prepFrame();
};

#endif /* TSL1401_H_ */

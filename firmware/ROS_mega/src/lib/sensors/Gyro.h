/*
 * Gyro.h
 *
 *  Created on: Apr 18, 2011
 *      Author: rbtying
 */

#include <Arduino.h>
#include "devices/CD74HC4067.h"

#ifndef GYRO_H_
#define GYRO_H_

#define LPR510_CONVERSION_FACTOR (0.034088462)
#define NUM_CAL 100

struct gyro_data {
	double rate;
	double val;
};

typedef struct gyro_data gyro_data;

class Gyro {
public:
	Gyro(uint8_t axisPin, uint8_t refPin, float conversion_factor);
	Gyro(CD74HC4067 *mux, uint8_t axisPin, uint8_t refPin,
			float conversion_factor);
	void calibrate(uint16_t num = NUM_CAL, bool continuous = false);
	float getValue();
	void update(gyro_data * d, float dt);
private:
	CD74HC4067 *m_mux;
	bool m_usingMux;
	uint8_t m_axisPin, m_refPin;
	float m_offset;
	float m_conversion_factor;
};

#endif /* GYRO_H_ */

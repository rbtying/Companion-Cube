/*
 * MaxSonar.h
 *
 *  Created on: Nov 20, 2010
 *      Author: rbtying
 */

#ifndef MAXSONAR_H_
#define MAXSONAR_H_
#include "WProgram.h"

class MaxSonar {
public:
	MaxSonar(uint8_t pin);
	uint8_t get_dist();
private:
	uint8_t m_pin;
};

#endif /* MAXSONAR_H_ */

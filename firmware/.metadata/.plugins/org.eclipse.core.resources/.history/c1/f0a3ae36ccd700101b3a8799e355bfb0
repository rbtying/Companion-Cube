/*
 * Sharp_IR.h
 *
 *  Created on: Nov 20, 2010
 *      Author: rbtying
 */
#include "WProgram.h"
#include "Rangefinder.h"

#ifndef SHARP_IR_H_
#define SHARP_IR_H_

#define SR 0
#define MR 1
#define LR 2

class Sharp_IR: Rangefinder {
public:
	Sharp_IR(uint8_t pin, uint8_t type);
	double get_dist();
private:
	uint8_t m_pin, m_type;
};

#endif /* SHARP_IR_H_ */

/*
 * Ping.h
 *
 *  Created on: Feb 12, 2011
 *      Author: rbtying
 */

#ifndef PING_H_
#define PING_H_
#include "WProgram.h"

class Ping {
public:
	Ping(uint8_t pin);
	uint16_t getDistance();

private:
	uint8_t m_pin;
};

#endif /* PING_H_ */

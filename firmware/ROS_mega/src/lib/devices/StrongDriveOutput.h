/*
 * StrongDriveOutput.h
 *
 *  Created on: Dec 10, 2011
 *      Author: rbtying
 */

#include "WProgram.h"

#ifndef STRONGDRIVEOUTPUT_H_
#define STRONGDRIVEOUTPUT_H_

class StrongDriveOutput {
public:
	StrongDriveOutput(uint8_t pin, bool inverted = false);
	void on();
	void off();
	void set(bool b);
private:
	volatile uint8_t *m_port;
	uint8_t m_pin;
	bool m_inverted;
};

#endif /* STRONGDRIVEOUTPUT_H_ */

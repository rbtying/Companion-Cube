/*
 * RC_CTRL.h
 *
 *  Created on: Feb 16, 2011
 *      Author: rbtying
 */

#ifndef RC_CTRL_H_
#define RC_CTRL_H_

#include <WProgram.h>

class RC_CTRL {
public:
	RC_CTRL(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5);
	uint8_t getChannel(uint8_t channel);
	int8_t getThrottle(bool left = true, uint8_t deadband = 10);
	int8_t getYaw(bool left = true, uint8_t deadband = 10);
	uint8_t getKnob();

	void update();
private:
	int8_t m_expo[5];
	uint8_t m_channels[5];
	uint8_t m_pins[5];
	uint8_t readRC(uint8_t channel);
	int8_t deadBandProcessor(int8_t val, uint8_t deadband);
};

#endif /* RC_CTRL_H_ */

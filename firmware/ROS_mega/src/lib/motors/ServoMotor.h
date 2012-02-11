/*
 * ServoMotor.h
 *
 *  Created on: Sep 5, 2011
 *      Author: rbtying
 */

#ifndef SERVOMOTOR_H_
#define SERVOMOTOR_H_

#include "DualMotor.h"
#include "Servo.h"

class ServoMotor: DualMotor {
public:
	ServoMotor(uint8_t left, uint8_t right);
	void setSpeed(motor_data * m);

private:
	Servo m_left, m_right;
};

#endif /* SERVOMOTOR_H_ */

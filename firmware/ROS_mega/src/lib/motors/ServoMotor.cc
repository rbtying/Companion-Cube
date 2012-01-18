/*
 * ServoMotor.cpp
 *
 *  Created on: Sep 5, 2011
 *      Author: rbtying
 */

#include "ServoMotor.h"

ServoMotor::ServoMotor(uint8_t left, uint8_t right) {
	m_left.attach(left);
	m_right.attach(right);
}

void ServoMotor::setSpeed(motor_data * m) {
	if (m->leftFwd) {
		m_left.writeMicroseconds(map(constrain(m->leftSpeed, -127, 127), -127,
				127, 1000, 2000));
	} else {
		m_left.writeMicroseconds(map(constrain(m->leftSpeed, -127, 127), -127,
				127, 2000, 1000));
	}

	if (m->rightFwd) {
		m_right.writeMicroseconds(map(constrain(m->rightSpeed, -127, 127),
				-127, 127, 1000, 2000));
	} else {
		m_right.writeMicroseconds(map(constrain(m->rightSpeed, -127, 127),
				-127, 127, 2000, 1000));
	}
}

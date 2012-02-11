/*
 * control_struct.h
 *
 *  Created on: Jul 17, 2011
 *      Author: rbtying
 */

#ifndef CONTROL_STRUCT_H_
#define CONTROL_STRUCT_H_

#include "Servo.h"
#include "sensors/Battery.h"

#define NUM_SERVOS 2

struct Motor {
	uint32_t p;
	uint32_t i;
	uint32_t d;
	double qp_to_m;
	int32_t count;
	int32_t vel;
};

struct LED {
	float r, g, b;
};

struct control_data {
	Battery mot_batt;
	Servo servos[NUM_SERVOS];
	Motor left;
	Motor right;
	struct {
		LED front;
		LED left;
		LED right;
		LED back;
	} LED;
	bool enabled;
};

typedef struct control_data control_data;

#endif /* CONTROL_STRUCT_H_ */

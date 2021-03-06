/*
 * Encoders.h
 *
 *  Created on: Sep 5, 2011
 *      Author: rbtying
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

struct encoder_data {
	volatile long count;
	volatile long pCount;

	volatile unsigned long time;
	volatile unsigned long pTime;

	volatile bool dir;
	volatile double velocity;

	double cmPerCount;
};

typedef struct encoder_data encoder_data;

class Encoder {
public:
	static void count(encoder_data * d) {
		d->count += d->dir ? +1 : -1;
		d->pTime = d->time;
		d->time = micros();
	}
	static void update(encoder_data * d, float dt) {
		d->velocity = (d->count - d->pCount) * d->cmPerCount / dt;
		d->pCount = d->count;
	}
};

#endif /* ENCODERS_H_ */

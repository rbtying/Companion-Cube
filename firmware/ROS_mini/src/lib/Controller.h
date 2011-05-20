/*
 * Controller.h
 *
 *  Created on: Feb 21, 2011
 *      Author: rbtying
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "WProgram.h"
#include "PowerMonitor.h"
#include "libraries/Servo/Servo.h"

#define CMD_BUFFER_SIZE 8
#define CMD_STARTCHAR ':'
#define CMD_ENDCHAR '!'
#define CMD_TIMEOUT 1000

class Controller {

public:
	Controller(double *lSet, double *rSet, double *lMeas, double *rMeas,
			long *lEnc, long *rEnc, PowerMonitor *batt, double *yawRate,
			double *yawVal, Servo *pan, Servo *tilt);
	void update();
	void printInfo();

	bool comm;

private:
	double *m_lSet, *m_rSet, *m_lMeas, *m_rMeas, *m_yawRate, *m_yawVal;
	PowerMonitor *m_batt;
	Servo *m_tilt, *m_pan;
	long *m_lEnc, *m_rEnc;

	unsigned long m_lastUpdateTime;
	char m_buf[CMD_BUFFER_SIZE];
	uint8_t m_bufPtr;
	void processCommand();
	char nextByte(unsigned long timeout);
	void flush();
};

#endif /* CONTROLLER_H_ */
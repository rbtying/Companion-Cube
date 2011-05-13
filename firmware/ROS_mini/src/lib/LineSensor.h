/*
 * LineSensor.h
 *
 *  Created on: Apr 10, 2011
 *      Author: rbtying
 */

#include "NewSoftSerial.h"
#include "WProgram.h"

#ifndef LINESENSOR_H_
#define LINESENSOR_H_

#define LS_BAUD_RATE 19200

#define LS_CMD_CALIBRATE_ON 'c'
#define LS_CMD_CALIBRATE_OFF 'o'
#define LS_CMD_READLINE 'l'
#define LS_CMD_READSENS 'r'
#define CMD_HUMAN 'h'

class LineSensor: NewSoftSerial {
public:
	LineSensor(uint8_t rxPin, uint8_t txPin);
	void setCalibrate(bool on);
	int16_t getLinePos();
	void getSensorValues(uint16_t * sensor_values);
private:
	uint8_t nextByte(unsigned long timeout);
};

#endif /* LINESENSOR_H_ */

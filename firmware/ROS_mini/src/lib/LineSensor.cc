/*
 * LineSensor.cpp
 *
 *  Created on: Apr 10, 2011
 *      Author: rbtying
 */

#include "WProgram.h"
#include "LineSensor.h"

LineSensor::LineSensor(uint8_t rxPin, uint8_t txPin) :
	NewSoftSerial(rxPin, txPin) {
	NewSoftSerial::begin(LS_BAUD_RATE);

}
void LineSensor::setCalibrate(bool on) {
	if (on)
		NewSoftSerial::print(LS_CMD_CALIBRATE_ON);
	else
		NewSoftSerial::print(LS_CMD_CALIBRATE_OFF);
}
int16_t LineSensor::getLinePos() {
	NewSoftSerial::print(LS_CMD_READLINE);
	byte msg[2];
	msg[0] = nextByte(100);
	msg[1] = nextByte(100);
	uint16_t val = (msg[0] << 8) | msg[1];
	return val - 3500;
}

void LineSensor::getSensorValues(uint16_t * sensor_values) {
	NewSoftSerial::print(LS_CMD_READSENS);
	byte msg[2];
	for (uint8_t i = 0; i < 8; i++) {
		msg[0] = nextByte(100);
		msg[1] = nextByte(100);
		sensor_values[i] = (msg[0] << 8) | msg[1];
	}
}

uint8_t LineSensor::nextByte(unsigned long timeout) {
	unsigned long timeOutTime = millis() + timeout;
	while (!NewSoftSerial::available() && millis() <= timeOutTime) {
		// do nothing
	}

	if (NewSoftSerial::available())
		return NewSoftSerial::read();
	else
		return 0xFF;
}

/*
 * Main.cpp
 *
 *  Created on: Nov 20, 2010
 *      Author: rbtying
 */
#include "WProgram.h"
#include "pins.h"
#include "lib/control_struct.h"
#include "lib/Controller.h"
#include "lib/PowerMonitor.h"
#include "lib/Gyro.h"
#include "lib/fastIO.h"
#include "lib/CD74HC4067.h"
#include "libraries/Servo/Servo.h"

// Control
#define TIME_INTERVAL 20 // 50 Hz
#define LED_INTERVAL 500

unsigned long nexTime = 0, cTime = 0, ledTime = 0;

// control data
control_data ctrl;

volatile long pleftEncCount, prightEncCount;
volatile bool leftEncBSet, rightEncBSet;
#define QP_TO_CM_LEFT (0.00199491134)
#define QP_TO_CM_RIGHT (0.00199491134)

int16_t lspeed, rspeed;

// adc mux:
CD74HC4067 mux(MUX_1, MUX_2, MUX_3, MUX_4, MUX_ADC);

// Yaw Gyroscope
Gyro yawGyro(&mux, YAW_GYRO, YAW_REF, LPR510_CONVERSION_FACTOR);
double yawVal = 0, yawRate = 0;

// Motors
Servo lMot, rMot;

Controller cmd(&ctrl);

/**
 * Sets speeds, +1000 to -1000
 */
void setSpeeds(int8_t left, int8_t right) {
	lMot.writeMicroseconds(map(constrain(left, -127, 127), -127, 127, 2000,
			1000));
	rMot.writeMicroseconds(map(constrain(right, -127, 127), -127, 127, 1000,
			2000));
}

/**
 * Encoder handlers
 */
void lEncHandler() {
	leftEncBSet = digitalReadFast(LENC_B);
#ifdef LENCREV
	ctrl.enc.leftCount -= leftEncBSet ? -1 : +1;
#else
	ctrl.enc.leftCount += leftEncBSet ? -1 : +1;
#endif
}

void rEncHandler() {
	rightEncBSet = digitalReadFast(RENC_B);
#ifdef LENCREV
	ctrl.enc.rightCount -= rightEncBSet ? -1 : +1;
#else
	ctrl.enc.rightCount += rightEncBSet ? -1 : +1;
#endif
}

/**
 * Main execution loop
 */
int main() {
	init();

	Serial.begin(115200);

	// battery
	ctrl.batt.set(VOLTAGE_SENS, CURRENT_SENS);

	// servos
	ctrl.pan.write(90);
	ctrl.pan.attach(PANSERVO);
	ctrl.tilt.write(90);
	ctrl.tilt.attach(TILTSERVO);

	// motors
	setSpeeds(0, 0);
	lMot.attach(LMOT);
	rMot.attach(RMOT);

	// PID
	ctrl.leftPID.proportional = 0.3;
	ctrl.leftPID.integral = 0.05;
	ctrl.leftPID.derivative = 0;

	ctrl.rightPID.proportional = 0.3;
	ctrl.rightPID.integral = 0.05;
	ctrl.rightPID.derivative = 0;

	ctrl.conv.cmPerCountLeft = QP_TO_CM_LEFT;
	ctrl.conv.cmPerCountRight = QP_TO_CM_RIGHT;

	// setup encoders
	uint8_t enc_pins[4] = { RENC_A, RENC_B, LENC_A, LENC_B };
	for (uint8_t i = 0; i < 4; i++) {
		pinModeFast(enc_pins[i], INPUT);
		digitalWriteFast(enc_pins[i], LOW);
	}
	attachInterrupt(LENC_INT, lEncHandler, RISING);
	attachInterrupt(RENC_INT, rEncHandler, RISING);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	yawGyro.calibrate(1000);

	for (uint32_t loops = 0;; loops++) {
		cmd.update();

		cTime = millis();

		if (ledTime <= cTime) {
			if (cmd.comm) {
				digitalWriteFast(LED, !digitalReadFast(LED));
			} else {
				digitalWriteFast(LED, LOW);
			}
			ledTime = cTime + LED_INTERVAL;
		}

		if (nexTime <= cTime) {
			float dt = (TIME_INTERVAL + (cTime - nexTime)) * 0.001;

			// update gyro
			ctrl.yaw.rate = yawGyro.getValue();
			ctrl.yaw.val += yawRate * dt;

			// correct for zero drift
			if (abs(ctrl.yaw.rate) <= 0.001 && ctrl.leftPID.set == 0
					&& ctrl.rightPID.set == 0) {// bot is not moving
				yawGyro.calibrate(1000, true); // update calibration
			}

			// PID processing
			ctrl.leftPID.input = (ctrl.enc.leftCount - pleftEncCount)
					* ctrl.conv.cmPerCountLeft / dt;
			ctrl.rightPID.input = (ctrl.enc.rightCount - prightEncCount)
					* ctrl.conv.cmPerCountRight / dt;
			pleftEncCount = ctrl.enc.leftCount;
			prightEncCount = ctrl.enc.rightCount;

			ctrl.leftPID.error = ctrl.leftPID.set - ctrl.leftPID.input;
			ctrl.rightPID.error = ctrl.rightPID.set - ctrl.rightPID.input;

			ctrl.leftPID.accumulated += ctrl.leftPID.error * dt;
			ctrl.rightPID.accumulated += ctrl.rightPID.error * dt;

			ctrl.leftPID.accumulated
					= constrain(ctrl.leftPID.accumulated, -127, 127);
			ctrl.rightPID.accumulated
					= constrain(ctrl.rightPID.accumulated, -127, 127);

			ctrl.leftPID.output = ctrl.leftPID.proportional * ctrl.leftPID.error
					+ ctrl.leftPID.integral * ctrl.leftPID.accumulated
					+ ctrl.leftPID.derivative * (ctrl.leftPID.error
							- ctrl.leftPID.previous) / dt;
			ctrl.rightPID.output = ctrl.rightPID.proportional
					* ctrl.rightPID.error + ctrl.rightPID.integral
					* ctrl.rightPID.accumulated + ctrl.rightPID.derivative
					* (ctrl.rightPID.error - ctrl.rightPID.previous) / dt;

			ctrl.leftPID.previous = ctrl.leftPID.error;
			ctrl.rightPID.previous = ctrl.rightPID.error;

			lspeed += ctrl.leftPID.output;
			rspeed += ctrl.rightPID.output;

			lspeed = constrain(lspeed, -127, 127);
			rspeed = constrain(rspeed, -127, 127);

			setSpeeds(lspeed & 0xff, rspeed & 0xff);

			nexTime = cTime + TIME_INTERVAL;
		}
	}
}

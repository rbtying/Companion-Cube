/*
 * Main.cpp
 *
 *  Created on: Nov 20, 2010
 *      Author: rbtying
 */
#include "WProgram.h"
#include "pins.h"
#include "lib/Controller.h"
#include "lib/PowerMonitor.h"
#include "lib/Gyro.h"
#include "lib/fastIO.h"
#include "lib/CD74HC4067.h"
#include "libraries/Servo/Servo.h"

// Control
#define TIME_INTERVAL 100
#define LED_INTERVAL 500
unsigned long nexTime = 0, cTime = 0, ledTime = 0;

double lSet, rSet, lMeasSpeed, rMeasSpeed;
double lacc, racc, lprev, rprev, lout, rout;
double lPID[3] = { .3, .05, 0 }, rPID[3] = { .3, 0.05, 0 }; // PID

volatile long leftEncCount, rightEncCount, pleftEncCount, prightEncCount;
volatile bool leftEncBSet, rightEncBSet;
#define QPPS_TO_CMPS_LEFT (0.004249148)
#define QPPS_TO_CMPS_RIGHT (0.004249148)

int16_t lspeed, rspeed;

// adc mux:
CD74HC4067 mux(MUX_1, MUX_2, MUX_3, MUX_4, MUX_ADC);

// Battery
PowerMonitor batt(VOLTAGE_SENS, CURRENT_SENS);

// Yaw Gyroscope
Gyro yawGyro(&mux, YAW_GYRO, YAW_REF, LPR510_CONVERSION_FACTOR);
double yawVal = 0, yawRate = 0;

// Servos
Servo panServo, tiltServo;

// Motors
Servo lMot, rMot;

Controller cmd(&lSet, &rSet, &lMeasSpeed, &rMeasSpeed, (long*) (&leftEncCount),
		(long*) (&rightEncCount), &batt, &yawRate, &yawVal, &panServo,
		&tiltServo);

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
	leftEncCount -= leftEncBSet ? -1 : +1;
#else
	leftEncCount += leftEncBSet ? -1 : +1;
#endif
}

void rEncHandler() {
	rightEncBSet = digitalReadFast(RENC_B);
#ifdef LENCREV
	rightEncCount -= rightEncBSet ? -1 : +1;
#else
	rightEncCount += rightEncBSet ? -1 : +1;
#endif
}

/**
 * Main execution loop
 */
int main() {
	init();

	Serial.begin(115200);

	// servos
	panServo.write(90);
	tiltServo.write(90);
	panServo.attach(PANSERVO);
	tiltServo.attach(TILTSERVO);

	// motors
	setSpeeds(0, 0);
	lMot.attach(LMOT);
	rMot.attach(RMOT);

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
			yawRate = yawGyro.getValue();
			yawVal += yawRate * dt;

			// correct for zero drift
			if (abs(yawRate) <= 0.001 && lSet == 0 && rSet == 0) {// bot is not moving
				yawGyro.calibrate(1000, true); // update calibration
			}

			// PID processing
			double lerr, rerr;
			lerr = lSet - lMeasSpeed;
			rerr = rSet - rMeasSpeed;

			lMeasSpeed = (leftEncCount - pleftEncCount) * QPPS_TO_CMPS_LEFT
					/ dt;
			rMeasSpeed = (rightEncCount - prightEncCount) * QPPS_TO_CMPS_RIGHT
					/ dt;
			pleftEncCount = leftEncCount;
			prightEncCount = rightEncCount;
			lacc += lerr * dt;
			racc += rerr * dt;

			lacc = constrain(lacc, -127, 127);
			racc = constrain(racc, -127, 127);

			lout = lPID[0] * lerr + lPID[1] * lacc + lPID[2] * ((lerr - lprev)
					/ (TIME_INTERVAL * 0.001));
			rout = rPID[0] * rerr + rPID[1] * racc + rPID[2] * ((rerr - rprev)
					/ (TIME_INTERVAL * 0.001));

			lprev = lerr;
			rprev = rerr;

			lspeed += lout;
			rspeed += rout;

			lspeed = constrain(lspeed, -127, 127);
			rspeed = constrain(rspeed, -127, 127);

			setSpeeds(lspeed & 0xff, rspeed & 0xff);

			//			Serial.print("left: ");
			//			Serial.print(lMeasSpeed, 2);
			//			Serial.print(" right: ");
			//			Serial.print(rMeasSpeed, 2);
			//			Serial.print(" lcount: ");
			//			Serial.print(leftEncCount);
			//			Serial.print(" rcount: ");
			//			Serial.print(rightEncCount);
			//			Serial.print(" lspd: ");
			//			Serial.print(lspeed);
			//			Serial.print(" rspd: ");
			//			Serial.print(rspeed);
			//			Serial.print("\r\n");

			nexTime = cTime + TIME_INTERVAL;
		}
	}
}

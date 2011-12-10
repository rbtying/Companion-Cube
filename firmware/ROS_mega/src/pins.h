/*
 * pins.h
 *
 *  Created on: Dec 8, 2010
 *      Author: rbtying
 */

#ifndef PINS_H_
#define PINS_H_

//  megamini Arduino

// ======= START ENCODERS =======
// encoder pins
#define LENC_A 18
#define LENC_B 19
#define RENC_A 20
#define RENC_B 21
#define RENCREV 1 // right-side encoder is reversed
// encoder interrupts
#define LENC_INT 5
#define RENC_INT 3
// ======= END ENCODERS =========

// ======= START MOTORS =========
#define SBT_ADDRESS 128
#define SBT_PIN1 14 // Serial3 Tx
#define SBT_PIN2 15 // Serial3 Rx
#define RELAY_PIN 42 // motor enabled relay
// ======= END MOTORS ===========

// ======= START SERVOS =========
#define PANSERVO 22
#define TILTSERVO 23
// ======= END SERVOS ===========

// ======= START BATT ===========
#define MOTOR_VOLTAGE_SENS 10
#define MOTOR_CURRENT_SENS 11
// ======= END BATT =============

// ======= START GYRO ===========
#define YAW_GYRO 9
#define YAW_REF 8
// ======= END GYRO =============

#endif /* PINS_H_ */

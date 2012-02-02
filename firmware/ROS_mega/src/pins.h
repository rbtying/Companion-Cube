/*
 * pins.h
 *
 *  Created on: Dec 8, 2010
 *      Author: rbtying
 */

#ifndef PINS_H_
#define PINS_H_

//  megamini Arduino

// ======= START MOTORS =========
#define RB_ADDRESS 128
#define RB_PIN1 18 // Serial1 Tx
#define RB_PIN2 19 // Serial1 Rx
#define RB_MAX_QPPS 9350
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

// ======= START IMU ============
#define IMU_POWER_PIN 49
// ======= END IMU ==============

#endif /* PINS_H_ */

/*
 * pins.h
 *
 *  Created on: Dec 8, 2010
 *      Author: rbtying
 */

#ifndef PINS_H_
#define PINS_H_

//  megamini Arduino
//
// cable:
// 0: GND (red)
// 1: Vcc
// 2: 10: MUX1
// 3: 11: MUX2
// 4: 12: MUX3
// 5: 13: MUX4
// 6: 14: ADC0
// 7:  8: PAN
// 8:  9: TILT
// 9: A3: LED

// ======= START LCD ============
#define LCD_LINES 4
#define LCD_COLS 20
#define LCD_RS 30
#define LCD_RW 31
#define LCD_EN 32
#define LCD_D4 33
#define LCD_D5 34
#define LCD_D6 35
#define LCD_D7 36
// ======= END LCD ==============

// ======= START ENCODERS =======
// encoder pins
#define LENC_A 2
#define LENC_B 7
#define RENC_A 3
#define RENC_B 8
// encoder interrupts
#define LENC_INT 0
#define RENC_INT 1
// ======= END ENCODERS =========

// ======= START MOTORS =========
#define SBT_ADDRESS 128
#define SBT_PIN1 14 // Serial3 Tx
#define SBT_PIN2 15 // Serial3 Rx
// ======= END MOTORS ===========

// ======= START SERVOS =========
#define PANSERVO 24
#define TILTSERVO 25
// ======= END SERVOS ===========

// ======= START MUX ============
#define MUX_1 26
#define MUX_2 27
#define MUX_3 28
#define MUX_4 29
#define MUX_ADC 0
// ======= END MUX ==============

// ======= START BATT ===========
#define VOLTAGE_SENS 15
#define CURRENT_SENS 14
#define MOTOR_VOLTAGE_SENS 13
#define MOTOR_CURRENT_SENS 12
// ======= END BATT =============

// ======= START GYRO ===========
#define YAW_GYRO 14
#define YAW_REF 15
// ======= END GYRO =============

// ======= START LED ============
#define LED 23
// ======= END LED ==============

#endif /* PINS_H_ */
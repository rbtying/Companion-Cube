/*
 * Main.cpp
 *
 *  Created on: September 4th, 2011
 *      Author: rbtying
 */
#include <WProgram.h>
#include "ros.h"
#include "rover/Enabled.h"
#include "rover/Motors.h"
#include "rover/Battery.h"
#include "rover/Encoder.h"
#include "rover/Settings.h"
#include "std_msgs/UInt8MultiArray.h"
#include "rover/Gyro.h"
#include "pins.h"
#include "control_struct.h"
#include "sensors/Gyro.h"
#include "sensors/Battery.h"
#include "utilities/fastIO.h"
#include "utilities/PID.h"
#include "motors/Sabertooth.h"
#include "devices/CD74HC4067.h"
#include "devices/StrongDriveOutput.h"
#include "libraries/Servo/Servo.h"

// Control
#define TIME_INTERVAL 100 // 10 Hz
#define COMM_INTERVAL 50 // 20 Hz
#define LED_INTERVAL 500

unsigned long nexTime = 0, cTime = 0, ledTime = 0;

// control data
control_data ctrl;

#define QP_TO_CM_LEFT (0.004)
#define QP_TO_CM_RIGHT (0.004)

// PID
PID leftPID(&ctrl.leftPID);
PID rightPID(&ctrl.rightPID);

// Yaw Gyroscope
Gyro yawGyro(YAW_GYRO, YAW_REF, LPR510_CONVERSION_FACTOR);

// Motors
Sabertooth m(SBT_ADDRESS, &Serial3);
StrongDriveOutput motorRelay(RELAY_PIN);

// Controller
//StateController cmd(&ctrl, &Serial);

// ROS
ros::NodeHandle nh;

rover::Enabled enablemsg;
ros::Publisher enablepub("enabled", &enablemsg);

rover::Encoder encdata;
ros::Publisher encpub("encoders", &encdata);

rover::Settings settingsdata;
ros::Publisher settingspub("settings_curr", &settingsdata);

rover::Gyro yawgyrodata;
ros::Publisher yawgyropub("yaw/gyro", &yawgyrodata);

rover::Battery batterydata;
ros::Publisher batterypub("battery/motor", &batterydata);

std_msgs::UInt8MultiArray servodata;
ros::Publisher servopub("servos_curr", &servodata);

// globals for message publication
char frameid[] = "base_footprint";
uint32_t seq = 0;
uint32_t nextCommTime;
bool publishSettingsDump = false;

/**
 * Encoder handlers
 */
void lEncHandler() {
#ifndef LENCREV
	ctrl.leftEnc.dir = !fastIORead(LENC_B);
#else
	ctrl.leftEnc.dir = fastIORead(LENC_B);
#endif
	Encoder::count(&ctrl.leftEnc);
}

void rEncHandler() {
#ifndef RENCREV
	ctrl.rightEnc.dir = !fastIORead(RENC_B);
#else
	ctrl.rightEnc.dir = fastIORead(RENC_B);
#endif
	Encoder::count(&ctrl.rightEnc);
}

void drivecb(const rover::Motors& msg) {
	ctrl.leftPID.set = 100.0 * msg.left;
	ctrl.rightPID.set = 100.0 * msg.right;
}
ros::Subscriber<rover::Motors> drivesub("drive", &drivecb);

void settingscb(const rover::Settings& msg) {
	ctrl.leftPID.proportional = msg.left_proportional;
	ctrl.leftPID.integral = msg.left_integral;
	ctrl.leftPID.derivative = msg.left_derivative;
	ctrl.leftEnc.cmPerCount = 100 * msg.left_conversion_factor;
	ctrl.rightPID.proportional = msg.right_proportional;
	ctrl.rightPID.integral = msg.right_integral;
	ctrl.rightPID.derivative = msg.right_derivative;
	ctrl.rightEnc.cmPerCount = 100 * msg.right_conversion_factor;
}
ros::Subscriber<rover::Settings> settingssub("settings", &settingscb);

void enablecb(const rover::Enabled& msg) {
	ctrl.enabled = msg.motorsEnabled;
	publishSettingsDump = msg.settingsDumpEnabled;
}
ros::Subscriber<rover::Enabled> enablesub("enable", &enablecb);

void servocb(const std_msgs::UInt8MultiArray& msg) {
	for (uint8_t i = 0; i < msg.data_length && i < NUM_SERVOS; i++) {
		ctrl.servos[i].write(msg.data[i]);
	}
}
ros::Subscriber<std_msgs::UInt8MultiArray> servosub("servos", &servocb);

void publish() {
	seq++;

	enablemsg.motorsEnabled = m.enabled();
	enablemsg.settingsDumpEnabled = publishSettingsDump;
	enablepub.publish(&enablemsg);

	for (uint8_t i = 0; i < NUM_SERVOS; i++) {
		servodata.data[i] = ctrl.servos[i].read();
	}
	servopub.publish(&servodata);

	encdata.header.frame_id = frameid;
	encdata.header.stamp = nh.now();
	encdata.header.seq = seq;
	encdata.left = ctrl.leftEnc.velocity / 100.0;
	encdata.leftCount = ctrl.leftEnc.count;
	encdata.leftMotor = ctrl.leftPID.output;
	encdata.left_conversion_factor = ctrl.leftEnc.cmPerCount / 100.0;
	encdata.right = ctrl.rightEnc.velocity / 100.0;
	encdata.rightCount = ctrl.rightEnc.count;
	encdata.rightMotor = ctrl.rightPID.output;
	encdata.right_conversion_factor = ctrl.rightEnc.cmPerCount / 100.0;
	encpub.publish(&encdata);

	yawgyrodata.header.frame_id = frameid;
	yawgyrodata.header.stamp = nh.now();
	yawgyrodata.header.seq = seq;
	yawgyrodata.rate = ctrl.yaw.rate;
	yawgyrodata.value = ctrl.yaw.val;
	yawgyropub.publish(&yawgyrodata);

	if (publishSettingsDump) {
		settingsdata.header.frame_id = frameid;
		settingsdata.header.stamp = nh.now();
		settingsdata.header.seq = seq;
		settingsdata.left_proportional = ctrl.leftPID.proportional;
		settingsdata.left_integral = ctrl.leftPID.integral;
		settingsdata.left_derivative = ctrl.leftPID.derivative;
		settingsdata.left_conversion_factor = ctrl.leftEnc.cmPerCount / 100.0;
		settingsdata.right_proportional = ctrl.rightPID.proportional;
		settingsdata.right_integral = ctrl.rightPID.integral;
		settingsdata.right_derivative = ctrl.rightPID.derivative;
		settingsdata.right_conversion_factor = ctrl.rightEnc.cmPerCount / 100.0;
		settingspub.publish(&settingsdata);
	}

	batterydata.header.frame_id = frameid;
	batterydata.header.stamp = nh.now();
	batterydata.header.seq = seq;
	batterydata.voltage = ctrl.mot_batt.getVoltage();
	batterydata.current = ctrl.mot_batt.getCurrent();
	batterypub.publish(&batterydata);
}

/**
 * Main execution loop
 */
int main() {
	init();

	nh.getHardware()->setBaud(250000);
	nh.initNode();
	//	Serial.begin(115200);

	nh.advertise(enablepub);
	nh.advertise(encpub);
	nh.advertise(settingspub);
	nh.advertise(yawgyropub);
	nh.advertise(batterypub);

	servodata.data = (uint8_t *) malloc(sizeof(uint8_t) * NUM_SERVOS);
	servodata.data_length = NUM_SERVOS;
	nh.advertise(servopub);

	nh.subscribe(enablesub);
	nh.subscribe(settingssub);
	nh.subscribe(drivesub);
	nh.subscribe(servosub);

	// battery
	ctrl.mot_batt.set(NULL, MOTOR_VOLTAGE_SENS, MOTOR_CURRENT_SENS);

	// PID
	ctrl.leftPID.proportional = 0.6;
	ctrl.leftPID.integral = 0.1;
	ctrl.leftPID.derivative = 0;
	ctrl.leftPID.accLimit = 127;

	ctrl.rightPID.proportional = 0.6;
	ctrl.rightPID.integral = 0.1;
	ctrl.rightPID.derivative = 0;
	ctrl.rightPID.accLimit = 127;

	ctrl.leftEnc.cmPerCount = QP_TO_CM_LEFT;
	ctrl.rightEnc.cmPerCount = QP_TO_CM_RIGHT;

	// Motors
	ctrl.mot.leftFwd = false;
	ctrl.mot.rightFwd = false;

	ctrl.enabled = false;

	// setup encoders
	uint8_t enc_pins[4] = { RENC_A, RENC_B, LENC_A, LENC_B };
	for (uint8_t i = 0; i < 4; i++) {
		fastIOMode(enc_pins[i], INPUT);
		fastIOWrite(enc_pins[i], LOW);
	}
	attachInterrupt(LENC_INT, lEncHandler, RISING);
	attachInterrupt(RENC_INT, rEncHandler, RISING);

	yawGyro.calibrate(1000);

	uint32_t resetTime = 0;

	pinMode(13, OUTPUT);

	for (uint32_t loops = 0;; loops++) {
		cTime = millis();

		//		cmd.update();

		if (!m.enabled() && ctrl.enabled) { // just enabled
			motorRelay.on();
			if (resetTime == 0) {
				resetTime = cTime + 2000;
			}
			if (cTime >= resetTime) {
				m.enable();
				resetTime = 0;
			}
		} else if (m.enabled() && !ctrl.enabled) { // just disabled
			m.disable();
			motorRelay.off();
		}

		if (nextCommTime <= cTime) {
			publish();
			digitalWrite(13, nh.connected());
			nextCommTime = nextCommTime + COMM_INTERVAL;
		}

		if (nexTime <= cTime/* && ctrl.enabled*/) {
			float dt = (TIME_INTERVAL + (cTime - nexTime)) * 0.001;

			// update gyro
			yawGyro.update(&ctrl.yaw, dt);

			// Encoder processing
			Encoder::update(&ctrl.leftEnc, dt);
			Encoder::update(&ctrl.rightEnc, dt);

			if (ctrl.enabled) {
				leftPID.setInput(ctrl.leftEnc.velocity);
				rightPID.setInput(ctrl.rightEnc.velocity);

				leftPID.process(dt);
				rightPID.process(dt);

				// correct for zero drift
				if (abs(ctrl.yaw.rate) <= 0.001 && ctrl.leftEnc.velocity == 0
						&& ctrl.rightEnc.velocity == 0) {// bot is not moving
					yawGyro.calibrate(1000, true); // update calibration
				}

				ctrl.mot.leftSpeed += leftPID.getOutput();
				ctrl.mot.rightSpeed += rightPID.getOutput();

				ctrl.mot.leftSpeed = constrain(ctrl.mot.leftSpeed, -127, 127);
				ctrl.mot.rightSpeed = constrain(ctrl.mot.rightSpeed, -127, 127);

				m.setSpeed(&ctrl.mot);
			}

			nexTime = nexTime + TIME_INTERVAL;
		}

		if (!nh.connected()) {
			ctrl.enabled = false;
		}

		nh.spinOnce();
	}
}

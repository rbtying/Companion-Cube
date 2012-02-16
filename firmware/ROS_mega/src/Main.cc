/*
 * Main.cpp
 *
 *  Created on: September 4th, 2011
 *      Author: rbtying
 */
#include <Arduino.h>
#include <Servo.h>

#include "pins.h"
#include "control_struct.h"

#include <ros.h>
#include <rover_msgs/Enabled.h>
#include <rover_msgs/Motors.h>
#include <rover_msgs/Battery.h>
#include <rover_msgs/Encoder.h>
#include <rover_msgs/Settings.h>
#include <rover_msgs/CondensedIMU.h>
#include <std_msgs/UInt8MultiArray.h>

#include <sensors/Gyro.h>
#include <sensors/Battery.h>
#include <sensors/imu/IMU.h>
#include <motors/RoboClaw.h>
#include <devices/StrongDriveOutput.h>
#include <devices/ShiftBrite.h>

// Control
#define TIME_INTERVAL 100 // 10 Hz
#define COMM_INTERVAL 40 // 25 Hz
#define LED_INTERVAL 10 // 100 Hz
unsigned long nexTime = 0, cTime = 0, ledTime = 0;

// control data
control_data ctrl;

// Shiftbrite
ShiftBrite sb(MOSI, LATCH, SS, SCK);
uint32_t nextLEDTime;

// Motors
RoboClaw m(&Serial1, 38400);
StrongDriveOutput motorRelay(RELAY_PIN);

// ROS
ros::NodeHandle nh;

rover_msgs::Enabled enablemsg;
ros::Publisher enablepub("enabled", &enablemsg);

rover_msgs::Encoder encdata;
ros::Publisher encpub("encoders", &encdata);

rover_msgs::Battery batterydata;
ros::Publisher batterypub("battery/motor", &batterydata);

rover_msgs::CondensedIMU imudata;
ros::Publisher imupub("imu/raw", &imudata);

std_msgs::UInt8MultiArray servodata;
ros::Publisher servopub("servos_curr", &servodata);

// globals for message publication
char frameid[] = "base_link";
uint32_t seq = 0;
uint32_t nextCommTime;
bool publishSettingsDump = false;

int32_t leftMotorSetting, rightMotorSetting;

void drivecb(const rover_msgs::Motors& msg) {
	leftMotorSetting = (int32_t)(msg.left / ctrl.left.qp_to_m);
	rightMotorSetting = (int32_t)(msg.right / ctrl.right.qp_to_m);
}
ros::Subscriber<rover_msgs::Motors> drivesub("drive", &drivecb);

void enablecb(const rover_msgs::Enabled& msg) {
	ctrl.enabled = msg.motorsEnabled;
	publishSettingsDump = msg.settingsDumpEnabled;
}
ros::Subscriber<rover_msgs::Enabled> enablesub("enable", &enablecb);

void servocb(const std_msgs::UInt8MultiArray& msg) {
	for (uint8_t i = 0; i < msg.data_length && i < NUM_SERVOS; i++) {
		ctrl.servos[i].write(msg.data[i]);
	}
}
ros::Subscriber<std_msgs::UInt8MultiArray> servosub("servos", &servocb);

void setMotor() {
	if (leftMotorSetting == 0) {
		m.SpeedM1(RB_ADDRESS, 1);
	} else {
		m.SpeedM1(RB_ADDRESS, leftMotorSetting);
	}

	if (rightMotorSetting == 0) {
		m.SpeedM2(RB_ADDRESS, 1);
	} else {
		m.SpeedM2(RB_ADDRESS, rightMotorSetting);
	}
	m.getSerial()->flush();
}

void publish() {
	seq++;

	enablemsg.motorsEnabled = ctrl.enabled;
	enablepub.publish(&enablemsg);

	batterydata.header.frame_id = frameid;
	batterydata.header.stamp = nh.now();
	batterydata.header.seq = seq;
	batterydata.voltage = ctrl.mot_batt.getVoltage();
	batterydata.current = ctrl.mot_batt.getCurrent();

	batterypub.publish(&batterydata);

	imudata.header.frame_id = frameid;
	imudata.header.stamp = nh.now();
	imudata.header.seq = seq;
	imudata.roll = IMU::roll;
	imudata.pitch = IMU::pitch;
	imudata.yaw = IMU::yaw;
	imudata.gyro_x = IMU::Gyro_Vector[0];
	imudata.gyro_y = IMU::Gyro_Vector[1];
	imudata.gyro_z = IMU::Gyro_Vector[2];
	imudata.accel_x = Accel_To_M_s_2(IMU::Accel_Vector[0]);
	imudata.accel_y = Accel_To_M_s_2(IMU::Accel_Vector[1]);
	imudata.accel_z = Accel_To_M_s_2(IMU::Accel_Vector[2]);
	imudata.mag_heading = (IMU::MAG_Heading);

	imupub.publish(&imudata);

	if (ctrl.enabled) {
		for (uint8_t i = 0; i < NUM_SERVOS; i++) {
			servodata.data[i] = ctrl.servos[i].read();
		}
		servopub.publish(&servodata);

		encdata.header.frame_id = frameid;
		encdata.header.stamp = nh.now();
		encdata.header.seq = seq;
		encdata.left = ctrl.left.vel * ctrl.left.qp_to_m;
		encdata.leftCount = ctrl.left.count;
		encdata.left_conversion_factor = ctrl.left.qp_to_m;
		encdata.right = ctrl.right.vel * ctrl.right.qp_to_m;
		encdata.rightCount = ctrl.right.count;
		encdata.right_conversion_factor = ctrl.right.qp_to_m;
		encpub.publish(&encdata);
	}
}

void setLEDs() {
	sb.setPower(127);
	sb.setPower(127);
	sb.setPower(127);
	sb.setPower(127);
#define rgb(x) (1023 * x.r), (1023 * x.g), (1023 * x.b)
	sb.setColor(rgb(ctrl.LED.left));
	sb.setColor(rgb(ctrl.LED.front));
	sb.setColor(rgb(ctrl.LED.right));
	sb.setColor(rgb(ctrl.LED.back));
#undef rgb
}

void setRGB(LED * s, double r, double g, double b) {
	s->r = r;
	s->g = g;
	s->b = b;
}

/**
 * Main execution loop
 */
int main() {
	init();

	IMU::initIMU();

	nh.getHardware()->setBaud(250000);
	nh.initNode();

	nh.advertise(enablepub);
	nh.advertise(encpub);
	nh.advertise(batterypub);
	nh.advertise(imupub);

	servodata.data = (uint8_t *) malloc(sizeof(uint8_t) * NUM_SERVOS);
	servodata.data_length = NUM_SERVOS;
	nh.advertise(servopub);

	nh.subscribe(enablesub);
	nh.subscribe(drivesub);
	nh.subscribe(servosub);

	ctrl.left.p = 0x00010000;
	ctrl.left.i = 0x00008000;
	ctrl.left.d = 0x00004000;
	ctrl.left.qp_to_m = 3.14159265 * 0.07265 / 1865;

	ctrl.right.p = 0x00010000;
	ctrl.right.i = 0x00008000;
	ctrl.right.d = 0x00004000;
	ctrl.right.qp_to_m = 3.14159265 * 0.07265 / 1865;

	// battery
	ctrl.mot_batt.set(NULL, MOTOR_VOLTAGE_SENS, MOTOR_CURRENT_SENS);

	// Motors
	ctrl.enabled = false;
	leftMotorSetting = 0;
	rightMotorSetting = 0;

	setRGB(&ctrl.LED.left, 0.3, 0.0, 0.0);
	setRGB(&ctrl.LED.right, 0.3, 0.0, 0.0);
	setRGB(&ctrl.LED.front, 0.3, 0.0, 0.0);
	setRGB(&ctrl.LED.back, 0.3, 0.0, 0.0);

	setLEDs();
	ledTime = 0;

	pinMode(13, OUTPUT);

	m.DutyM1M2(RB_ADDRESS, 0, 0);
	m.SetMinVoltageLogicBattery(RB_ADDRESS, 0x00);
	m.SetMinVoltageMainBattery(RB_ADDRESS, 0x00);
	m.SetM1Constants(RB_ADDRESS, ctrl.left.d, ctrl.left.p, ctrl.left.i,
			RB_MAX_QPPS);
	m.SetM2Constants(RB_ADDRESS, ctrl.right.d, ctrl.right.p, ctrl.right.i,
			RB_MAX_QPPS);

	for (uint32_t loops = 0;; loops++) {
		cTime = millis();

		//		if (IMU::updateIMU()) {
		//			IMU::printdata();
		//		}

		if (ctrl.enabled) {
			if (!motorRelay.get()) {
				motorRelay.on();
				m.SetMinVoltageLogicBattery(RB_ADDRESS, 0x00);
				m.SetMinVoltageMainBattery(RB_ADDRESS, 0x00);
				m.SetM1Constants(RB_ADDRESS, ctrl.left.d, ctrl.left.p,
						ctrl.left.i, RB_MAX_QPPS);
				m.SetM2Constants(RB_ADDRESS, ctrl.right.d, ctrl.right.p,
						ctrl.right.i, RB_MAX_QPPS);
			}
		} else {
			motorRelay.off();
			m.DutyM1M2(RB_ADDRESS, 0, 0);
			leftMotorSetting = 0;
			rightMotorSetting = 0;
		}

		if (nexTime <= cTime) {
			uint8_t status;
			bool valid;

			ctrl.left.count = m.ReadEncM1(RB_ADDRESS, &status, &valid);
			ctrl.right.count = m.ReadEncM2(RB_ADDRESS, &status, &valid);
			ctrl.left.vel = m.ReadISpeedM1(RB_ADDRESS, &status, &valid);
			ctrl.right.vel = m.ReadISpeedM2(RB_ADDRESS, &status, &valid);

			if (ctrl.enabled) {
				setMotor();
			} else {
				m.DutyM1M2(RB_ADDRESS, 0, 0);
			}

			nexTime = nexTime + TIME_INTERVAL;
		}

		if (nextCommTime <= cTime) {
			publish();
			digitalWrite(13, nh.connected());
			nextCommTime = nextCommTime + COMM_INTERVAL;
		}

		if (ledTime <= cTime) {
			static double fade = 0.0;
			static bool inc;
			if (fade >= 0.3) {
				inc = false;
			} else if (fade <= 0.0) {
				inc = true;
			}
			fade += ((inc) ? (1) : (-1)) * 0.01;
			if (!ctrl.enabled) {
				setRGB(&ctrl.LED.left, fade, 0.0, 0.0);
				setRGB(&ctrl.LED.right, fade, 0.0, 0.0);
				setRGB(&ctrl.LED.front, fade, 0.0, 0.0);
				setRGB(&ctrl.LED.back, fade, 0.0, 0.0);
			} else {
				setRGB(&ctrl.LED.left, 1.0, 0.0, 1.0);
				setRGB(&ctrl.LED.right, 1.0, 0.0, 1.0);
				setRGB(&ctrl.LED.front, 1.0, 1.0, 1.0);
				setRGB(&ctrl.LED.back, 1.0, 0.0, 0.0);

			}
			setLEDs();
			ledTime = ledTime + LED_INTERVAL;
		}

		if (!nh.connected()) {
			ctrl.enabled = false;
			m.DutyM1M2(RB_ADDRESS, 0, 0);
		}

		// run serial event loop
		nh.spinOnce();
		if (serialEventRun)
			serialEventRun();
	}
}

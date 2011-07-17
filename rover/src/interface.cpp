#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>

#include "interface.h"

rover::interface::interface(const char * new_serial_port) {
	m_port_name = new_serial_port;

	this->resetOdometry();

	m_lastSensorUpdateTime = ros::Time::now();

	m_port = new cereal::CerealPort();

	m_roverAxleLength = ROVER_DEFAULT_AXLE_LENGTH;

	boost::function<void(std::string*)> callback;
	callback = boost::bind(&rover::interface::processPacket, this, _1);
//	m_port->startReadLineStream(callback);
	m_port->startReadBetweenStream(callback, '<', '>');
}

rover::interface::~interface() {
	driveDirect(0, 0);
	// Clean up!
	delete m_port;
}

int rover::interface::openSerialPort() {
	try {
		m_port->open(m_port_name.c_str(), 115200);
	} catch (cereal::Exception& e) {
		return (-1);
	}

	char cmd[5];
	cmd[0] = ':';
	cmd[1] = 's';
	cmd[2] = 's';
	cmd[3] = '!';
	cmd[4] = 0x01;

	try {
		m_port->write(cmd, 5);
	} catch (cereal::Exception& e) {
		return (-1);
	}

	return (0);
}

int rover::interface::closeSerialPort() {
	this->drive(0.0, 0.0);

	char cmd[5];
	cmd[0] = ':';
	cmd[1] = 's';
	cmd[2] = 's';
	cmd[3] = '!';
	cmd[4] = 0x00;

	try {
		m_port->write(cmd, 5);
		m_port->close();
	} catch (cereal::Exception& e) {
		return (-1);
	}

	return (0);
}

int rover::interface::drive(double linear_speed, double angular_speed) {
	int leftSpeed_mm = (int) ((linear_speed - (m_roverAxleLength
			* angular_speed)) * 1000); // Left wheel velocity in mm/s
	int rightSpeed_mm = (int) ((linear_speed + (m_roverAxleLength
			* angular_speed)) * 1000); // Right wheel velocity in mm/s

	return this->driveDirect(leftSpeed_mm, rightSpeed_mm);
}

int rover::interface::driveDirect(int left_speed, int right_speed) {
	// limit velocities
	int16_t left_speed_mm = max(left_speed, -ROVER_MAX_VEL_MM);
	left_speed_mm = min(left_speed, ROVER_MAX_VEL_MM);
	int16_t right_speed_mm = max(right_speed, -ROVER_MAX_VEL_MM);
	right_speed_mm = min(right_speed, ROVER_MAX_VEL_MM);

	// ROS_INFO("Driving at [left: %i mm/s, right: %i mm/s]", left_speed_mm, right_speed_mm);

	// Compose comand
	char cmd_buffer[10];
	cmd_buffer[0] = START_CHAR;
	cmd_buffer[1] = 's';
	cmd_buffer[2] = 'D';
	cmd_buffer[3] = 'R';
	cmd_buffer[4] = 'V';
	cmd_buffer[5] = END_CHAR;
	cmd_buffer[6] = (char) (left_speed_mm >> 8); // high byte
	cmd_buffer[7] = (char) (left_speed_mm & 0xff); // low byte
	cmd_buffer[8] = (char) (right_speed_mm >> 8); // high byte
	cmd_buffer[9] = (char) (right_speed_mm & 0xff); // low byte

	try {
		m_port->write(cmd_buffer, 10);
	} catch (cereal::Exception& e) {
		return (-1);
	}

	return (0);
}

int rover::interface::setServos(double panAngle, double tiltAngle) {
	// limit angles to +- pi/2
	panAngle = max(panAngle, -HALF_PI);
	panAngle = min(panAngle, HALF_PI);
	tiltAngle = max(tiltAngle, -HALF_PI);
	tiltAngle = min(tiltAngle, HALF_PI);

	// convert to servo degrees (0-180)
	unsigned char panDeg = max((unsigned char) (panAngle * RAD_TO_DEG) + 90, 0);
	panDeg = min(panDeg, 180);
	unsigned char tiltDeg = max((unsigned char) (tiltAngle * RAD_TO_DEG) + 90, 0);
	tiltDeg = min(tiltDeg, 180);

	// ROS_INFO("Setting servos to pan: %i degrees, tilt: %i degrees", panDeg, tiltDeg);

	char cmd_buffer[8];
	cmd_buffer[0] = START_CHAR;
	cmd_buffer[1] = 's';
	cmd_buffer[2] = 'S';
	cmd_buffer[3] = 'E';
	cmd_buffer[4] = 'R';
	cmd_buffer[5] = END_CHAR;
	cmd_buffer[6] = (char) (panDeg & 0xFF);
	cmd_buffer[7] = (char) (tiltDeg & 0xFF);

	try {
		m_port->write(cmd_buffer, 8);
	} catch (cereal::Exception& e) {
		return (-1);
	}
	return (0);
}

void rover::interface::processPacket(std::string * packet) {
	uint8_t data[packet->size()];
	for(uint8_t i = 0; i < packet->size(); i++) {
		data[i] = static_cast<uint8_t>(packet->at(i));
	}

	if (packet->size() == 17) {
		ros::Time current_time = ros::Time::now();
		double dt = (current_time - m_lastSensorUpdateTime).toSec();

		uint16_t leftSpeed = data[1] << 8  | data[2];
		uint16_t rightSpeed = data[3] << 8 | data[4];
		uint16_t yawRate = data[5] << 8 | data[6];
		uint16_t yaw = data[7] << 8 | data[9];

		m_velocity_left = leftSpeed / 10000.0;
		m_velocity_right = rightSpeed / 10000.0;

		if (!(data[9] & (1 << 0)))
			m_velocity_left *= -1;
		if (!(data[9] & (1 << 1)))
			m_velocity_right *= -1;
		
		m_gyro_yawrate = yawRate / 1000.0;
		m_gyro_yaw = yaw / 1000.0;

		if (!(data[9] & (1 << 2)))
			m_gyro_yawrate *= -1;
		if (!(data[9] & (1 << 3)))
			m_gyro_yaw *= -1;

		m_gyro_yaw -= m_gyro_offset;

		m_battery_voltage = ((data[10] << 8 | data[11]) / 100.0);
		m_battery_current = ((data[12] << 8 | data[13]) / 100.0);

		m_pan_angle = data[14] * DEG_TO_RAD - HALF_PI; // convert to +- pi/2
		m_tilt_angle = data[15] * DEG_TO_RAD - HALF_PI; // convert to +- pi/2
	
		this->calculateOdometry(dt);
		m_lastSensorUpdateTime = current_time;
		newPacket = true;
	}
}

int rover::interface::getSensorPackets(int timeout) {
	m_port->flush();

	std::string in;

	try {
		m_port->readBetween(&in, '<', '>', timeout);
	} catch (cereal::Exception& e) {
		return 1;
	} catch (std::length_error& e) {
		return 2;
	}

	rover::interface::processPacket(&in);

	return 0;
}

void rover::interface::setPID(double lP, double lI, double lD, double rP, double rI, double rD) {
	int16_t lpt = (int16_t) (lP * 100);
	int16_t lit = (int16_t) (lI * 100);
	int16_t ldt = (int16_t) (lD * 100);

	int16_t rpt = (int16_t) (rP * 100);
	int16_t rit = (int16_t) (rI * 100);
	int16_t rdt = (int16_t) (rD * 100);

	char msg[18];
	msg[0] = ':';
	msg[1] = 's';
	msg[2] = 'P';
	msg[3] = 'I';
	msg[4] = 'D';
	msg[5] = '!';

	msg[6] = lpt >> 8;
	msg[7] = lpt & 0xff;

	msg[8] = lit >> 8;
	msg[9] = lit & 0xff;

	msg[10] = ldt >> 8;
	msg[11] = ldt & 0xff;

	msg[12] = rpt >> 8;
	msg[13] = rpt & 0xff;

	msg[14] = rit >> 8;
	msg[15] = rit & 0xff;

	msg[16] = rdt >> 8;
	msg[17] = rdt & 0xff;

	try {
		m_port->write(msg, 18);
	} catch (cereal::Exception& e) {
		
	}
}

void rover::interface::calculateOdometry(double dt) {
	double linSpeed = (m_velocity_left + m_velocity_right) / 2; // m/s
	this->m_velocity_yaw
			= normalize((m_velocity_right - m_velocity_left) / (m_roverAxleLength * 2)); // rad/s

	this->m_odometry_yaw
			= normalize(this->m_odometry_yaw + this->m_velocity_yaw * dt); // rad

	this->m_velocity_x = linSpeed * cos(this->m_odometry_yaw); // m/s
	this->m_velocity_y = linSpeed * sin(this->m_odometry_yaw); // m/s

	// Update odometry
	this->m_odometry_x = this->m_odometry_x + m_velocity_x * dt; // m
	this->m_odometry_y = this->m_odometry_y + m_velocity_y * dt; // m
}

void rover::interface::resetOdometry() {
	this->setOdometry(0.0, 0.0, 0.0);
}

void rover::interface::setOdometry(double new_x, double new_y, double new_yaw) {
	this->m_odometry_x = new_x;
	this->m_odometry_y = new_y;
	this->m_odometry_yaw = new_yaw;
	this->m_gyro_offset = m_gyro_yaw - new_yaw;
}

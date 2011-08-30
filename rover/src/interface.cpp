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
    	m_max_vel = ROVER_MAX_VEL_MM;

	boost::function<void(std::string*)> callback;
	callback = boost::bind(&rover::interface::processPacket, this, _1);
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
	int16_t left_speed_mm = max(left_speed, -(m_max_vel * 1000));
	left_speed_mm = min(left_speed, (m_max_vel * 1000 ));
	int16_t right_speed_mm = max(right_speed, -(m_max_vel * 1000));
	right_speed_mm = min(right_speed, (m_max_vel * 1000));

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

int rover::interface::setMotorsRaw(int8_t left, int8_t right) {
	char cmd_buffer[8];
	cmd_buffer[0] = START_CHAR;
	cmd_buffer[1] = 's';
	cmd_buffer[2] = 'M';
	cmd_buffer[3] = 'O';
	cmd_buffer[4] = 'T';
	cmd_buffer[5] = END_CHAR;
	cmd_buffer[6] = (char) (left & 0xFF);
	cmd_buffer[7] = (char) (right & 0xFF);

	try {
		m_port->write(cmd_buffer, 8);
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

	if (packet->size() == 27) {
		ros::Time current_time = ros::Time::now();
		double dt = (current_time - m_lastSensorUpdateTime).toSec();

        uint32_t leftCount, rightCount;
        uint16_t leftSpeed, rightSpeed, yawRate, yawVal;
        int16_t voltage, current;
        uint8_t sign, panAngle, tiltAngle;
        int8_t leftOutSpeed, rightOutSpeed;

#define SIGN_LEFT_VEL (1 << 0)
#define SIGN_RIGHT_VEL (1 << 1)
#define SIGN_YAW_RATE (1 << 2)
#define SIGN_YAW_VAL (1 << 3)
#define SIGN_LEFT_ENC (1 << 4)
#define SIGN_RIGHT_ENC (1 << 5)
#define SIGN(x) ((sign & (x)) ? (1) : (-1))

        sign = data[1];
        leftSpeed = data[2] << 8u | data[3];
        rightSpeed = data[4] << 8u | data[5];
        leftOutSpeed = data[6];
        rightOutSpeed = data[7];
        leftCount = data[8] << 24u | data[9] << 16u | data[10] << 8u | data[11];
        rightCount = data[12] << 24u | data[13] << 16u | data[14] << 8u | data[15];
        yawRate = data[16] << 8u | data[17];
        yawVal = data[18] << 8u | data[19];
        voltage = data[20] << 8u | data[21];
        current = data[22] << 8u | data[23];
        tiltAngle = data[24];
        panAngle = data[25];

        m_left_raw = leftOutSpeed;
        m_right_raw = rightOutSpeed;

		m_velocity_left = leftSpeed / 10000.0 * SIGN(SIGN_LEFT_VEL);
		m_velocity_right = rightSpeed / 10000.0 * SIGN(SIGN_RIGHT_VEL);

        m_encoder_left = leftCount * SIGN(SIGN_LEFT_ENC);
        m_encoder_right = rightCount * SIGN(SIGN_RIGHT_ENC);

        m_gyro_yawrate = yawRate / 1000.0 * m_gyro_correction * SIGN(SIGN_YAW_RATE);
        m_gyro_yaw += m_gyro_yawrate * dt - m_gyro_offset;
		
		m_battery_voltage = voltage / 100.0;
		m_battery_current = current / 100.0;

		m_pan_angle = panAngle * DEG_TO_RAD - HALF_PI; // convert to +- pi/2
		m_tilt_angle = tiltAngle * DEG_TO_RAD - HALF_PI; // convert to +- pi/2
	
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

void rover::interface::setConversionFactors(double left, double right) {
    int16_t left_conv = left * 10000;
    int16_t right_conv = right * 10000;

    char msg[11];
    msg[0] = ':';
    msg[1] = 's';
    msg[2] = 'C';
    msg[3] = 'O';
    msg[4] = 'N';
    msg[5] = 'V';
    msg[6] = '!';
    msg[7] = left_conv >> 8;
    msg[8] = left_conv & 0xff;
    msg[9] = right_conv >> 8;
    msg[10] = right_conv & 0xff;

    try {
        m_port->write(msg, 11);
    } catch (cereal::Exception& e) {

    }
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

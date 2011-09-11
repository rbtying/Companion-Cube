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

	m_lastEncoderUpdateTime = ros::Time::now();
    m_lastYawGyroUpdateTime = m_lastEncoderUpdateTime;

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

	return (0);
}

int rover::interface::closeSerialPort() {
	this->drive(0.0, 0.0);

	try {
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
	char cmd_buffer[5];
    cmd_buffer[0] = CTRL_OP_SET_DRIVE;
	cmd_buffer[1] = (char) (left_speed_mm >> 8); // high byte
	cmd_buffer[2] = (char) (left_speed_mm & 0xff); // low byte
	cmd_buffer[3] = (char) (right_speed_mm >> 8); // high byte
	cmd_buffer[4] = (char) (right_speed_mm & 0xff); // low byte

	try {
		m_port->write(cmd_buffer, 5);
	} catch (cereal::Exception& e) {
		return (-1);
	}

	return (0);
}

int rover::interface::setMotorsRaw(int8_t left, int8_t right) {
	char cmd_buffer[3];
	cmd_buffer[0] = CTRL_OP_SET_MOTOR;
	cmd_buffer[1] = (char) (left & 0xFF);
	cmd_buffer[2] = (char) (right & 0xFF);

	try {
		m_port->write(cmd_buffer, 3);
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

	char cmd_buffer[3];
	cmd_buffer[0] = CTRL_OP_SET_SERVO;
	cmd_buffer[1] = (char) (panDeg & 0xFF);
	cmd_buffer[2] = (char) (tiltDeg & 0xFF);

	try {
		m_port->write(cmd_buffer, 3);
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

    ros::Time current_time = ros::Time::now();

    int16_t speed, rate, val, voltage, current;
    int32_t count;
    int8_t out;
    uint8_t angle;

    switch(data[1]) {
    case MSG_OP_LEFT_ENC:
        if (packet->size() == 11) {
            speed = data[2] << 8 | data[3];
            count = data[4] << 24u | data[5] << 16u | data[6] << 8 | data[7];
            out = data[8];

            m_velocity_left = speed / 10000.0;
            m_encoder_left = count;
            newLeftEncPacket = true;
        } else {
            ROS_ERROR("Left encoder packet failed");
        }
        break;
    case MSG_OP_RIGHT_ENC:
        if (packet->size() == 11) {
            speed = data[2] << 8 | data[3];
            count = data[4] << 24u | data[5] << 16u | data[6] << 8 | data[7];
            out = data[8];

            m_velocity_right = speed / 10000.0;
            m_encoder_right = count;
            newRightEncPacket = true;
        } else {
            ROS_ERROR("Right encoder packet failed");
        }
        break;
    case MSG_OP_YAW_GYRO:
        if (packet->size() == 8) {
            double dt = (current_time - m_lastYawGyroUpdateTime).toSec();
            rate = data[2] << 8 | data[3];
            val = data[4] << 8 | data[5];

            m_gyro_yawrate = rate / 1000.0 * m_gyro_correction;
            m_gyro_yaw += m_gyro_yawrate * dt - m_gyro_offset;
            m_lastYawGyroUpdateTime = current_time;
        } else {
            ROS_ERROR("Yaw gyro packet failed");
        }
        break;
    case MSG_OP_CPU_BAT:
        if (packet->size() == 8) {
            voltage = data[2] << 8 | data[3];
            current = data[4] << 8 | data[5];

            m_battery_voltage = voltage / 100.0;
            m_battery_current = current / 100.0;
        } else {
            ROS_ERROR("CPU battery packet failed");
        }
        break;
    case MSG_OP_MOTOR_BAT:
        if (packet->size() == 8) {
            voltage = data[2] << 8 | data[3];
            current = data[4] << 8 | data[5];

            m_motor_voltage = voltage / 100.0;
            m_motor_current = current / 100.0;
        } else {
            ROS_ERROR("Motor battery packet failed");
        }
        break;
    case MSG_OP_PAN_SERVO:
        if (packet->size() == 5) {
            angle = data[2];
            m_pan_angle = angle * DEG_TO_RAD - HALF_PI; // convert to +- pi/2
        } else {
            ROS_ERROR("Pan servo packet failed");
        }
        break;
    case MSG_OP_TILT_SERVO:
        if (packet->size() == 5) {
            angle = data[2];
            m_tilt_angle = angle * DEG_TO_RAD - HALF_PI; // convert to +- pi/2
        } else {
            ROS_ERROR("Tilt servo packet failed");
        }
        break;
    }
    
    if (newLeftEncPacket && newRightEncPacket) {
        double dt = (current_time - m_lastEncoderUpdateTime).toSec();
        this->calculateOdometry(dt);
        newLeftEncPacket = false;
        newRightEncPacket = false;
    }
    newPacket = true;
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

void rover::interface::setLCD(std::string text, int lineNum) {
    char msg[22];
    msg[0] = CTRL_OP_SET_LCD;
    msg[1] = (uint8_t) lineNum;
    if (text.length() < 20) {
        text.append(20 - text.length(), ' ');
    }
    for (int i = 0; i < 20; i++) {
        msg[i + 2] = text[i];
    }
    try {
        m_port->write(msg, 22);
    } catch (cereal::Exception& e) {

    }
}

void rover::interface::setConversionFactors(double left, double right) {
    int16_t left_conv = left * 10000;
    int16_t right_conv = right * 10000;

    char msg[5];
    msg[0] = CTRL_OP_SET_CONV;
    msg[1] = left_conv >> 8;
    msg[2] = left_conv & 0xff;
    msg[3] = right_conv >> 8;
    msg[4] = right_conv & 0xff;

    try {
        m_port->write(msg, 5);
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

	char msg[13];
    msg[0] = CTRL_OP_SET_PID;
	msg[1] = lpt >> 8;
	msg[2] = lpt & 0xff;
	msg[3] = lit >> 8;
	msg[4] = lit & 0xff;
	msg[5] = ldt >> 8;
	msg[6] = ldt & 0xff;
	msg[7] = rpt >> 8;
	msg[8] = rpt & 0xff;
	msg[9] = rit >> 8;
	msg[10] = rit & 0xff;
	msg[11] = rdt >> 8;
	msg[12] = rdt & 0xff;

	try {
		m_port->write(msg, 13);
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

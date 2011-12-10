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

    m_port = new cereal::CerealPort();

    m_roverAxleLength = ROVER_DEFAULT_AXLE_LENGTH;
    m_max_vel = ROVER_MAX_VEL_MM;

    m_last_time = ros::Time::now();

    boost::function<void(std::string*)> callback;
    callback = boost::bind(&rover::interface::processPacket, this, _1);
    m_port->startReadBetweenStream(callback, '<', '>');
}

rover::interface::~interface() {
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
    this->sendData();

    try {
        m_port->close();
    } catch (cereal::Exception& e) {
        return (-1);
    }

    return (0);
}

int rover::interface::sendData() {
    char cmd[STATE_STRUCT_SIZE + 2];
    uint8_t buf[STATE_STRUCT_SIZE];
    memset(&m_state, 0, STATE_STRUCT_SIZE);
    memset(cmd, 0, STATE_STRUCT_SIZE + 2);
    memset(buf, 0, STATE_STRUCT_SIZE);

    // populate robot state based on given data
    // PID
    m_state.pid_left_proportional = m_ds.lP;
    m_state.pid_left_integral = m_ds.lI;
    m_state.pid_left_derivative = m_ds.lD;
    m_state.pid_left_setpoint = m_ds.left_speed;

    m_state.pid_right_proportional = m_ds.rP;
    m_state.pid_right_integral = m_ds.rI;
    m_state.pid_right_derivative = m_ds.rD;
    m_state.pid_right_setpoint = m_ds.right_speed;

    // servos
    m_state.servo_pan_val = m_ds.pan_angle;
    m_state.servo_tilt_val = m_ds.tilt_angle;

    // conversion factors
    m_state.enc_left_conv = m_ds.cfl;
    m_state.enc_right_conv = m_ds.cfr;

    // flags
    m_state.flag = m_ds.flag;

    // convert robot state into byte*
    stateStructToByte(&m_state, buf);

    // copy values into proper buffer
    for (int i = 0; i < STATE_STRUCT_SIZE; i++) {
        if (buf[i] == '<' || buf[i] == '>') {
            cmd[i+1] = '='; // ensure that no start or end characters
                            // end up in the output
        } else {
            cmd[i+1] = (char) buf[i];
        }
    }
    cmd[0] = '<';
    cmd[STATE_STRUCT_SIZE + 1] = '>';

    try {
        m_port->write(cmd, STATE_STRUCT_SIZE + 2);
    } catch (cereal::Exception& e) {
        return (-1);
    }

    return (0);
}

void rover::interface::enable() {
    m_ds.flag |= 1 << FLAG_MOTOR_ENABLED;
    sendData();
}

void rover::interface::disable() {
    m_ds.flag &= ~(1 << FLAG_MOTOR_ENABLED);
    sendData();
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

    m_ds.left_speed = left_speed_mm;
    m_ds.right_speed = right_speed_mm;

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

    m_ds.pan_angle = (uint8_t) panDeg;
    m_ds.tilt_angle = (uint8_t) tiltDeg;

    return (0);
}

void rover::interface::processPacket(std::string * packet) {
    uint8_t data[packet->size()];
    for(uint8_t i = 0; i < packet->size(); i++) {
        data[i] = static_cast<uint8_t>(packet->at(i));
    }

    if (packet->size() == STATE_STRUCT_SIZE + 2) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - m_last_time).toSec();
        robot_state rs;
        uint8_t buf[STATE_STRUCT_SIZE + 1];
        for (int i = 0; i < STATE_STRUCT_SIZE + 1; i++) {
            buf[i] = data[i+1];
        }
        // process byte array and fill state struct
        byteToStateStruct(buf, &rs);

        // process state struct and fill variables
        // encoders
        m_velocity_left = rs.enc_left_speed / 10000.0;
        m_encoder_left = rs.enc_left_count;
        m_left_raw = rs.motor_left_val;

        m_velocity_right = rs.enc_right_speed / 10000.0;
        m_encoder_right = rs.enc_right_count;
        m_right_raw = rs.motor_right_val;

        // gyro
        m_gyro_yawrate = rs.gyro_yaw_rate / 1000.0 * m_gyro_correction;
        m_gyro_yaw += m_gyro_yawrate * dt - m_gyro_offset;

        // battery
        m_motor_voltage = rs.batt_motor_voltage / 100.0;
        m_motor_current = rs.batt_motor_current / 100.0;

        // servos
        m_tilt_angle = rs.servo_tilt_val * DEG_TO_RAD - HALF_PI;
        m_pan_angle = rs.servo_pan_val * DEG_TO_RAD - HALF_PI;

        // calculate odometry
        this->calculateOdometry(dt);

        // reset time interval for dt
        m_last_time = current_time;
        newPacket = true;
    } else {
        ROS_ERROR("Packet length error: %d", (int) packet->size());
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
    m_ds.cfl = left * 10000;
    m_ds.cfr = right * 10000;
}

void rover::interface::setPID(double lP, double lI, double lD, double rP, double rI, double rD) {
    m_ds.lP = (int16_t) (lP * 100);
    m_ds.lI = (int16_t) (lI * 100);
    m_ds.lD = (int16_t) (lD * 100);

    m_ds.rP = (int16_t) (rP * 100);
    m_ds.rI = (int16_t) (rI * 100);
    m_ds.rD = (int16_t) (rD * 100);
}

void rover::interface::calculateOdometry(double dt) {
    double linSpeed = (m_velocity_left + m_velocity_right) / 2; // m/s

    this->m_velocity_yaw = normalize((m_velocity_right - m_velocity_left) / (m_roverAxleLength * 2)); // rad/s
    this->m_odometry_yaw = normalize(this->m_odometry_yaw + this->m_velocity_yaw * dt); // rad

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

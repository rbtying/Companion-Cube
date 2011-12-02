#ifndef INTERFACE_H_
#define INTERFACE_H_

#include <ros/ros.h>
#include <string>
#include "state_struct.h"
#include "cereal_port/CerealPort.h"
#include "util.h"

// dimensions
#define ROVER_DEFAULT_AXLE_LENGTH 0.240164856

#define ROVER_MAX_VEL_MM 500

#define QPPS_TO_M_S 9.78134873e-6

namespace rover {

struct desired_state {
    int16_t left_speed;
    int16_t right_speed;
    uint8_t pan_angle;
    uint8_t tilt_angle;
    int16_t cfl;
    int16_t cfr;
    int16_t lP;
    int16_t lI;
    int16_t lD;
    int16_t rP;
    int16_t rI;
    int16_t rD;
};

typedef struct desired_state desired_state;

class interface {
public:

	interface(const char * new_serial_port);
	~interface();

	int openSerialPort();
	int closeSerialPort();

	int getSensorPackets(int timeout);

	int drive(double linear_speed, double angular_speed);
	int driveDirect(int left_speed, int right_speed);

	int setServos(double panAngle, double tiltAngle);

    void setConversionFactors(double left, double right);
	void setPID(double lP, double lI, double lD, double rP, double rI, double rD);
    
	void resetOdometry();
	void setOdometry(double new_x, double new_y, double new_yaw);

    int sendData();

	double m_odometry_x;
	double m_odometry_y;
	double m_odometry_yaw;
	double m_gyro_yaw;
	double m_velocity_x;
	double m_velocity_y;
	double m_velocity_yaw;
	double m_gyro_yawrate;
	double m_gyro_offset;

    double m_motor_voltage;
    double m_motor_current;
	
    double m_velocity_left, m_velocity_right;
    long m_encoder_left, m_encoder_right;

    int8_t m_left_raw;
    int8_t m_right_raw;

	double m_tilt_angle;
	double m_pan_angle;

	double m_roverAxleLength;
    double m_max_vel;
    double m_gyro_correction;

	bool newPacket;
    bool newLeftEncPacket;
    bool newRightEncPacket;
    bool newEncPacket;
private:
    ros::Time m_last_time;
	std::string m_port_name;
	cereal::CerealPort * m_port;
    robot_state m_state;
    desired_state m_ds;
	void calculateOdometry(double dt);
	void processPacket(std::string * packet);
};

}

#endif

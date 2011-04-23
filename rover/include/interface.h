#include <ros/ros.h>
#include "cereal_port/CerealPort.h"

// dimensions
#define ROVER_DEFAULT_AXLE_LENGTH 0.240164856

#define ROVER_MAX_VEL_MM 500

#define QPPS_TO_M_S 9.78134873e-6

#define START_CHAR ':'
#define END_CHAR '!'

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif
#ifndef ABS
#define ABS(x) ((x < 0) ? (-x) : (x));
#endif

namespace rover {
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

	void resetOdometry();
	void setOdometry(double new_x, double new_y, double new_yaw);

	double m_odometry_x;
	double m_odometry_y;
	double m_odometry_yaw;
	double m_gyro_yaw;
	double m_velocity_x;
	double m_velocity_y;
	double m_velocity_yaw;
	double m_gyro_yawrate;
	double m_gyro_offset;
	double m_battery_voltage;
	double m_battery_current;
	double m_velocity_left, m_velocity_right;

	double m_tilt_angle;
	double m_pan_angle;

	double m_roverAxleLength;
private:
	long m_last_enc_left, m_last_enc_right;
	ros::Time m_lastSensorUpdateTime;
	std::string m_port_name;
	cereal::CerealPort * m_port;
	void calculateOdometry(double dt);
};

}

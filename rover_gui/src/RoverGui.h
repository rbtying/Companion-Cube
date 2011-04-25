#include "util.h"

class RoverGui {
public:
	RoverGui();
	void update();
	void loop();
private:
	void kinectCallback(const std_msgs::Float64::ConstPtr& angle);
	void panCallback(const std_msgs::Float64::ConstPtr& angle);
	void tiltCallback(const std_msgs::Float64::ConstPtr& angle);
	void battCallback(const rover::Battery::ConstPtr& batt);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

	ros::NodeHandle m_n;
	
	double m_kinect_angle, m_pan_angle, m_tilt_angle, m_batt_voltage;
	double m_velocity, m_angular_velocity;

	ros::Time lastUpdateTime;

	ros::Subscriber m_kinect_sub;
	ros::Subscriber m_pan_sub;
	ros::Subscriber m_tilt_sub;
	ros::Subscriber m_batt_sub;
	ros::Subscriber m_odom_sub;
};


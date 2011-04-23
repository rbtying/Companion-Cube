#include <ros/ros.h>
#include <std_msgs/Float64.h>		// for servos
#include <rover/Battery.h>		// for the battery
#include <nav_msgs/Odometry.h>		// for speed

#define min(x, y) ((x < y) ? (x) : (y))
#define max(x, y) ((x > y) ? (x) : (y))
#define constrain(x, y, z) (min(max(x, y), z))

#define scale(x, in_min, in_max, out_min, out_max) ((constrain(x, in_min, in_max) - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define expo(x) ((x) < (0)) ? ((-1) * (x) * (x)) : ((x) * (x))

#define HALF_PI 1.57079633
#define PI 3.14159265

class RoverGui {
public:
	RoverGui();
	void update();
	void loop();
private:
	void kinectCallback(const std_msgs::Float64::ConstPtr& angle);
	void panCallback(const std_msgs::Float64::ConstPtr& angle);
	void tiltCallback(const std_msgs::Float64::ConstrPtr& angle);
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

RoverGui::RoverGui()
{
	m_kinect_sub = m_n.subscribe<std_msgs::Float64>("kinect/cur_angle", 1, &RoverGui::kinectCallback, this);
	m_pan_sub = m_n.subscribe<std_msgs::Float64>("pan/cur_angle", 1, &RoverGui::panCallback, this);
	m_tilt_sub = m_n.subscribe<std_msgs::Float64>("tilt/cur_angle", 1, &RoverGui::tiltCallback, this);
	m_batt_sub = m_n.subscribe<rover::Battery>("battery", 1, &RoverGui::battCallback, this);
	m_odom_sub = m_n.subscribe<nav_msgs::Odometry>("odom", 1, &RoverGui::odomCallback, this);
}

void RoverGui::loop() {
	ros::Rate r(20.0);
	while (n.ok()) {
		update();
		ros::spinOnce();
		r.sleep();
	}
}

void RoverGui::update() {
	ROS_INFO("Angles: [%.02f] [%.02f] [%.02f] Velocity: [%.02f] [%.02f] Battery: [%.02f]"
		, m_kinect_angle, m_pan_angle, m_tilt_angle
		, m_velocity, m_angular_velocity
		, m_batt_voltage 
		);
}

void RoverGui::kinectCallback(const std_msgs::Float64::ConstPtr& angle)
{
	m_kinect_angle = angle->data;
}

void RoverGui::panCallback(const std_msgs::Float64::ConstPtr& angle)
{
	m_pan_angle = angle->data;
}

void RoverGui::tiltCallback(const std_msgs::Float64::ConstrPtr& angle)
{
	m_tilt_angle = angle->data;
}

void RoverGui::battCallback(const rover::Battery::ConstPtr& batt) 
{
	m_batt_voltage = batt->voltage;
}

void RoverGui::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	m_velocity = odom->linear.x;
	m_angular_velocity = odom->angular.z;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "rover_gui");
	
	RoverGui gui;

	gui.loop();

	ros::spin();
}

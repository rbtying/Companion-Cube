#include <std_msgs/Float64.h>		// for servos
#include <rover/Battery.h>		// for the battery
#include <nav_msgs/Odometry.h>		// for speed
#include "RoverGui.h"

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
	while (m_n.ok()) {
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

void RoverGui::tiltCallback(const std_msgs::Float64::ConstPtr& angle)
{
	m_tilt_angle = angle->data;
}

void RoverGui::battCallback(const rover::Battery::ConstPtr& batt) 
{
	m_batt_voltage = batt->voltage;
}

void RoverGui::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	m_velocity = odom->twist.twist.linear.x;
	m_angular_velocity = odom->twist.twist.angular.z;
}

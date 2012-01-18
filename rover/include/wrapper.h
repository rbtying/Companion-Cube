/*
 * wrapper.h
 *
 *  Created on: Jan 17, 2012
 *      Author: rbtying
 */

#ifndef WRAPPER_H_
#define WRAPPER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt8MultiArray.h>	// servos
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <sensor_msgs/Imu.h>			// imu
#include <sensor_msgs/JointState.h>		// joints
#include <rover/Battery.h>			    // battery
#include <rover/Encoder.h>			    // encoders
#include <rover/Settings.h>             // settings
#include <rover/Gyro.h>                 // gyro
#include <rover/Motors.h>				// motors
#include <rover/Enabled.h>              // enablemessage
//#include "servomapping.h"
#include "util.h"
#include <string>

class Wrapper {
public:
	Wrapper();
	~Wrapper();

private:
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	void battCallback(const rover::Battery::ConstPtr& bat);
	void encCallback(const rover::Encoder::ConstPtr& enc);
	void gyrCallback(const rover::Gyro::ConstPtr& gyr);
	void servoCallback(const std_msgs::UInt8MultiArray::ConstPtr& serv);
	void setSettings(double lp, double li, double ld, double lc, double rp,
			double ri, double rd, double rc);
	ros::NodeHandle m_n;

	double m_axlelength, m_maxvel;
	double m_minBatVoltage;

	ros::Time m_previous_odom_calc_time;
	ros::Time m_previous_gyro_calc_time;

	int32_t m_previous_left_count;
	int32_t m_previous_right_count;

	double m_odom_x;
	double m_odom_y;
	double m_odom_yaw;
	double m_vel_x;
	double m_vel_y;
	double m_vel_yaw;

	double m_yaw_gyro_rate;
	double m_yaw_gyro_value;
	double m_yaw_gyro_correction_factor;

	bool m_publish_tf;

	std::string m_odom_frame_id;

	// publishers
	ros::Publisher m_odom_pub;
	ros::Publisher m_imu_pub;
	ros::Publisher m_motors_pub;
	ros::Publisher m_settings_pub;
	ros::Publisher m_servos_pub;
	ros::Publisher m_joint_pub;
	ros::Publisher m_enable_pub;
	tf::TransformBroadcaster m_odom_broadcaster;

	// subscribers
	ros::Subscriber m_cmd_vel_sub;
	ros::Subscriber m_batt_sub;
	ros::Subscriber m_enc_sub;
	ros::Subscriber m_gyro_sub;
	ros::Subscriber m_servos_sub;
};

#endif /* WRAPPER_H_ */

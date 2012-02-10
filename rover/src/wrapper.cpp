#include "wrapper.h"
#include <string>

Wrapper::~Wrapper() {
	rover_msgs::Enabled msg;
	msg.motorsEnabled = false;
	m_enable_pub.publish(msg);
}

Wrapper::Wrapper() :
	m_n("~") {
	m_odom_pub = m_n.advertise<nav_msgs::Odometry> ("odom", 1);
	m_imu_pub = m_n.advertise<sensor_msgs::Imu> ("imu/data", 1);
	m_enable_pub = m_n.advertise<rover_msgs::Enabled> ("enable", 1, true);
	m_motors_pub = m_n.advertise<rover_msgs::Motors> ("drive", 1);
	m_servos_pub = m_n.advertise<std_msgs::UInt8MultiArray> ("servos", 1);
	m_joint_pub = m_n.advertise<sensor_msgs::JointState> ("joint_states", 1);

	m_cmd_vel_sub = m_n.subscribe<geometry_msgs::Twist> ("cmd_vel", 10,
			&Wrapper::cmdVelCallback, this);
	m_batt_sub = m_n.subscribe<rover_msgs::Battery> ("battery/motor", 10,
			&Wrapper::battCallback, this);
	m_enc_sub = m_n.subscribe<rover_msgs::Encoder> ("encoders", 10,
			&Wrapper::encCallback, this);
	m_imu_sub = m_n.subscribe<rover_msgs::CondensedIMU> ("imu/raw", 10,
			&Wrapper::imuCallback, this);
	m_servos_sub = m_n.subscribe<std_msgs::UInt8MultiArray> ("servos_curr", 10,
			&Wrapper::servoCallback, this);

	// parameters
	m_n.param<double> ("axleLength", m_axlelength, 0.235);
	m_n.param<double> ("maxSpeed", m_maxvel, 0.75);

	m_n.param<double> ("motor_battery_threshold", m_minBatVoltage, 8.5);

	m_n.param<std::string> ("odom_frame_id", m_odom_frame_id, "odom");
	m_n.param<bool> ("publish_tf", m_publish_tf, true);

	//	usleep(5e6);

	rover_msgs::Enabled msg;
	msg.motorsEnabled = true;
	m_enable_pub.publish(msg);

	std_msgs::UInt8MultiArray servomsg;
	servomsg.data.resize(2);
	servomsg.data[0] = HALF_PI * RAD_TO_DEG;
	servomsg.data[1] = HALF_PI * RAD_TO_DEG;
	m_servos_pub.publish(servomsg);
}

void Wrapper::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	rover_msgs::Motors m;
	m.left = cmd_vel->linear.x - m_axlelength * cmd_vel->angular.z;
	m.right = cmd_vel->linear.x + m_axlelength * cmd_vel->angular.z;
	m_motors_pub.publish(m);
}

void Wrapper::battCallback(const rover_msgs::Battery::ConstPtr& bat) {
	if (bat->voltage < m_minBatVoltage) {
		ROS_ERROR("Motor battery voltage low: %.02f volts, %.02f amps"
				, bat->voltage, bat->current);
	}
}

void Wrapper::imuCallback(const rover_msgs::CondensedIMU::ConstPtr& imu) {
	// create a quaternion for the gyro
#define radians(x) ((x) * DEG_TO_RAD)
	geometry_msgs::Quaternion imu_quat =
			tf::createQuaternionMsgFromRollPitchYaw(radians(imu->roll),
					radians(imu->pitch), radians(imu->yaw));

	// create a sensor msg for the gyro
	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp = ros::Time::now();
	imu_msg.header.frame_id = "base_link";

	imu_msg.orientation = imu_quat;
	imu_msg.orientation_covariance[0] = 1e6;
	imu_msg.orientation_covariance[4] = 1e6;
	imu_msg.orientation_covariance[8] = 1e-6;

	imu_msg.angular_velocity.x = imu->gyro_x;
	imu_msg.angular_velocity.y = imu->gyro_y;
	imu_msg.angular_velocity.z = imu->gyro_z;
	imu_msg.angular_velocity_covariance[0] = 1e6;
	imu_msg.angular_velocity_covariance[4] = 1e6;
	imu_msg.angular_velocity_covariance[8] = 1e-6;

	imu_msg.linear_acceleration.x = imu->accel_x;
	imu_msg.linear_acceleration.y = imu->accel_y;
	imu_msg.linear_acceleration.z = imu->accel_z;
	imu_msg.linear_acceleration_covariance[0] = 1e6;
	imu_msg.linear_acceleration_covariance[4] = 1e6;
	imu_msg.linear_acceleration_covariance[8] = 1e-6;

	// send the message
	m_imu_pub.publish(imu_msg);
}

void Wrapper::servoCallback(const std_msgs::UInt8MultiArray::ConstPtr& serv) {
	// prepare a joint state message
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();

	joint_state.name.resize(serv->data.size());
	joint_state.position.resize(serv->data.size());

	std::string names[] = { "pan_servo_joint", "tilt_servo_joint" };

	for (uint8_t i = 0; i < serv->data.size(); i++) {
		joint_state.name[i] = names[i];
		joint_state.position[i] = serv->data[i] * DEG_TO_RAD - HALF_PI;
	}
	m_joint_pub.publish(joint_state);
}

void Wrapper::encCallback(const rover_msgs::Encoder::ConstPtr& enc) {
	if (m_p_enc_msg != NULL) {
		double dt = (enc->header.stamp - m_p_enc_msg->header.stamp).toSec();

		double dl = (enc->leftCount - m_p_enc_msg->leftCount)
				* enc->left_conversion_factor;
		double dr = (enc->rightCount - m_p_enc_msg->rightCount)
				* enc->right_conversion_factor;

		double dth = (dl - dr) / (m_axlelength * 2);

		m_vel_yaw = normalize(dth/dt); // rad/s
		m_odom_yaw = normalize(m_odom_yaw + dth); // rad

		double dvect = (dl + dr) / 2;
		double dx = dvect * cos(m_odom_yaw);
		double dy = dvect * sin(m_odom_yaw);

		m_vel_x = dx / dt;
		m_vel_y = dy / dt;

		// Update odometry
		m_odom_x += dx; // m
		m_odom_y += dy; // m

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
				m_odom_yaw);

		if (m_publish_tf) {
			// create a tf message
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = ros::Time::now();
			odom_trans.header.frame_id = m_odom_frame_id.c_str();
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = m_odom_x;
			odom_trans.transform.translation.y = m_odom_y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			m_odom_broadcaster.sendTransform(odom_trans);
		}

		// create a odometry msg
		nav_msgs::Odometry odom;
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = m_odom_frame_id.c_str();

		// set the position
		odom.pose.pose.position.x = m_odom_x;
		odom.pose.pose.position.y = m_odom_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = m_vel_x;
		odom.twist.twist.linear.y = m_vel_y;
		odom.twist.twist.angular.z = m_vel_yaw;

		// set the covariance
		double covariance[36] = { 1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0,
				0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0,
				0, 0, 0, 1e3 };

		for (int i = 0; i < 36; i++) {
			odom.pose.covariance[i] = covariance[i];
			odom.twist.covariance[i] = covariance[i];
		}

		// publish the message
		m_odom_pub.publish(odom);
	}
	// save this message for later
	m_p_enc_msg = enc;
}

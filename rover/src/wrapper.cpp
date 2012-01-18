#include "wrapper.h"
#include <string>

Wrapper::~Wrapper() {
	rover::Enabled msg;
	msg.motorsEnabled = false;
	m_enable_pub.publish(msg);
}

Wrapper::Wrapper() :
	m_n("~") {
	m_odom_pub = m_n.advertise<nav_msgs::Odometry> ("odom", 1);
	m_imu_pub = m_n.advertise<sensor_msgs::Imu> ("imu", 1);
	m_enable_pub = m_n.advertise<rover::Enabled> ("enable", 1, true);
	m_motors_pub = m_n.advertise<rover::Motors> ("drive", 1);
	m_settings_pub = m_n.advertise<rover::Settings> ("settings", 1, true);
	m_servos_pub = m_n.advertise<std_msgs::UInt8MultiArray> ("servos", 1);
	m_joint_pub = m_n.advertise<sensor_msgs::JointState> ("joint_states", 1);

	m_cmd_vel_sub = m_n.subscribe<geometry_msgs::Twist> ("cmd_vel", 10,
			&Wrapper::cmdVelCallback, this);
	m_batt_sub = m_n.subscribe<rover::Battery> ("battery/motor", 10,
			&Wrapper::battCallback, this);
	m_enc_sub = m_n.subscribe<rover::Encoder> ("encoders", 10,
			&Wrapper::encCallback, this);
	m_gyro_sub = m_n.subscribe<rover::Gyro> ("yaw/gyro", 10,
			&Wrapper::gyrCallback, this);
	m_servos_sub = m_n.subscribe<std_msgs::UInt8MultiArray> ("servos_curr", 10,
			&Wrapper::servoCallback, this);

	double lP, lI, lD, rP, rI, rD, le, re;

	// parameters
	m_n.param<double> ("axleLength", m_axlelength, 0.10);
	m_n.param<double> ("maxSpeed", m_maxvel, 0.75);

	m_n.param<double> ("motor_battery_threshold", m_minBatVoltage, 8.5);

	m_n.param<double> ("left/proportional", lP, 0.3);
	m_n.param<double> ("left/integral", lI, 0.05);
	m_n.param<double> ("left/derivative", lD, 0);

	m_n.param<double> ("right/proportional", rP, 0.3);
	m_n.param<double> ("right/integral", rI, 0.05);
	m_n.param<double> ("right/derivative", rD, 0);

	m_n.param<double> ("left/conversion_factor", le, 0.00004);
	m_n.param<double> ("right/conversion_factor", re, 0.00004);

	m_n.param<std::string> ("odom_frame_id", m_odom_frame_id, "odom");
	m_n.param<bool> ("publish_tf", m_publish_tf, true);
	m_n.param<double> ("gyro_correction", m_yaw_gyro_correction_factor, 1.0);

	//	usleep(5e6);

	m_previous_gyro_calc_time = ros::Time::now();
	m_previous_odom_calc_time = ros::Time::now();

	setSettings(lP, lI, lD, le, rP, rI, rD, re);

	rover::Enabled msg;
	msg.motorsEnabled = true;
	m_enable_pub.publish(msg);

	std_msgs::UInt8MultiArray servomsg;
	servomsg.data.resize(2);
	servomsg.data[0] = HALF_PI * RAD_TO_DEG;
	servomsg.data[1] = HALF_PI * RAD_TO_DEG;
	m_servos_pub.publish(servomsg);
}

void Wrapper::setSettings(double lp, double li, double ld, double lc,
		double rp, double ri, double rd, double rc) {
	rover::Settings m;
	m.left_proportional = lp;
	m.left_integral = li;
	m.left_derivative = ld;
	m.left_conversion_factor = lc;
	m.right_proportional = rp;
	m.right_integral = ri;
	m.right_derivative = rd;
	m.right_conversion_factor = rc;
	m_settings_pub.publish(m);
}

void Wrapper::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	rover::Motors m;
	m.left = cmd_vel->linear.x - m_axlelength * cmd_vel->angular.z;
	m.right = cmd_vel->linear.x + m_axlelength * cmd_vel->angular.z;
	m_motors_pub.publish(m);
}

void Wrapper::battCallback(const rover::Battery::ConstPtr& bat) {
	if (bat->voltage < m_minBatVoltage) {
		ROS_ERROR("Motor battery voltage low: %.02f volts, %.02f amps"
				, bat->voltage, bat->current);
	}
}

void Wrapper::gyrCallback(const rover::Gyro::ConstPtr& gyr) {
	ros::Time current_time = ros::Time::now();
	double dt = (m_previous_gyro_calc_time - current_time).toSec();

	m_yaw_gyro_rate = gyr->rate * m_yaw_gyro_correction_factor;
	m_yaw_gyro_value += dt * m_yaw_gyro_rate;

	// create a quaternion for the gyro
	geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(
			m_yaw_gyro_value);

	// create a sensor msg for the gyro
	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp = current_time;
	imu_msg.header.frame_id = "base_footprint";

	imu_msg.orientation = imu_quat;
	imu_msg.orientation_covariance[0] = 1e6;
	imu_msg.orientation_covariance[4] = 1e6;
	imu_msg.orientation_covariance[8] = 1e-6;

	imu_msg.angular_velocity.x = 0.0;
	imu_msg.angular_velocity.y = 0.0;
	imu_msg.angular_velocity.z = m_yaw_gyro_rate;
	imu_msg.angular_velocity_covariance[0] = 1e6;
	imu_msg.angular_velocity_covariance[4] = 1e6;
	imu_msg.angular_velocity_covariance[8] = 1e-6;

	imu_msg.linear_acceleration.x = 0.0;
	imu_msg.linear_acceleration.y = 0.0;
	imu_msg.linear_acceleration.z = 0.0;
	imu_msg.linear_acceleration_covariance[0] = -1; // no accelerometers, so just set covariance to -1

	// send the message
	m_imu_pub.publish(imu_msg);
	m_previous_gyro_calc_time = current_time;
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

void Wrapper::encCallback(const rover::Encoder::ConstPtr& enc) {
	ros::Time current_time = ros::Time::now();
	double dt = (m_previous_odom_calc_time - current_time).toSec();

	double left_avg_vel = (enc->leftCount - m_previous_left_count)
			* enc->left_conversion_factor / dt;
	double right_avg_vel = (enc->rightCount - m_previous_right_count)
			* enc->right_conversion_factor / dt;

	double linSpeed = (left_avg_vel + right_avg_vel) / 2; // m/s

	m_vel_yaw = normalize((left_avg_vel - right_avg_vel) / (m_axlelength * 2)); // rad/s
	m_odom_yaw = normalize(m_odom_yaw + m_vel_yaw * dt); // rad

	m_vel_x = linSpeed * cos(m_odom_yaw);
	m_vel_y = linSpeed * sin(m_odom_yaw);

	// Update odometry
	m_odom_x += m_vel_x * dt; // m
	m_odom_y += m_vel_y * dt; // m

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
			m_odom_yaw);

	if (m_publish_tf) {
		// create a tf message
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = m_odom_frame_id.c_str();
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = m_odom_x;
		odom_trans.transform.translation.y = m_odom_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		m_odom_broadcaster.sendTransform(odom_trans);
	}

	// create a odometry msg
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = m_odom_frame_id.c_str();

	// set the position
	odom.pose.pose.position.x = m_odom_x;
	odom.pose.pose.position.y = m_odom_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// set the velocity
	odom.child_frame_id = "base_footprint";
	odom.twist.twist.linear.x = m_vel_x;
	odom.twist.twist.linear.y = m_vel_y;
	odom.twist.twist.angular.z = m_vel_yaw;

	// set the covariance
	double covariance[36] = { 1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0,
			1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0,
			0, 1e3 };

	for (int i = 0; i < 36; i++) {
		odom.pose.covariance[i] = covariance[i];
		odom.twist.covariance[i] = covariance[i];
	}

	// publish the message
	m_odom_pub.publish(odom);

	m_previous_left_count = enc->leftCount;
	m_previous_right_count = enc->rightCount;

	m_previous_odom_calc_time = current_time;
}
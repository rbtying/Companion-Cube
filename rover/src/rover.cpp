#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <sensor_msgs/Imu.h>			// imu
#include <sensor_msgs/JointState.h>		// joints
#include <std_msgs/Float64.h>			// floats
#include <rover/Battery.h>			// battery
#include <rover/Encoder.h>			// encoders
#include "interface.h"

#include <string>

std::string port;

rover::interface * bot;

double tilt_angle = 0;
double tilt_servo_angle = 0;
double pan_servo_angle = 0;

std::string prefixTopic(std::string prefix, char * name) {
	std::string topic_name = prefix;
	topic_name.append(name);

	return topic_name;
}

// callback functions
void recvKinectTiltAngle(const std_msgs::Float64::ConstPtr& tiltAngle) {
	tilt_angle = tiltAngle->data;
	// ROS_INFO("Kinect Tilt Angle: %f", tilt_angle);
}

void recvTiltServoAngle(const std_msgs::Float64::ConstPtr& tiltAngle) {
	tilt_servo_angle = tiltAngle->data;
	bot->setServos(-pan_servo_angle, -tilt_servo_angle);
	// ROS_INFO("Tilt Servo Angle: %f", tilt_servo_angle);
}

void recvPanServoAngle(const std_msgs::Float64::ConstPtr& panAngle) {
	pan_servo_angle = panAngle->data;
	bot->setServos(-pan_servo_angle, -tilt_servo_angle);
	// ROS_INFO("Pan Servo Angle: %f", pan_servo_angle);
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	bot->drive(cmd_vel->linear.x, cmd_vel->angular.z);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "rover_node");

	ros::NodeHandle n;

	n.param<std::string> ("rover/port", port, "/dev/ttyUSB0");

	bot = new rover::interface(port.c_str());

	double lP, lI, lD, rP, rI, rD, le, re;
	
	// parameters
	n.param<double> ("rover/axleLength", bot->m_roverAxleLength,
			bot->m_roverAxleLength);
	n.param<double> ("rover/maxSpeed", bot->m_max_vel,
            bot->m_max_vel);
	n.param<double> ("rover/left/proportional", lP, 0.3);
	n.param<double> ("rover/left/integral", lI, 0.05);
	n.param<double> ("rover/left/derivative", lD, 0);

	n.param<double> ("rover/right/proportional", rP, 0.3);
	n.param<double> ("rover/right/integral", rI, 0.05);
	n.param<double> ("rover/right/derivative", rD, 0);

    	n.param<double> ("rover/left/conversion_factor", le, 0.00199491134);
    	n.param<double> ("rover/right/conversion_factor", re, 0.00199491134);

	// publishers
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry> ("/odom", 50);
	ros::Publisher enc_pub = n.advertise<rover::Encoder> ("/encoders", 50);
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu> ("/gyro", 50);
	ros::Publisher batt_pub = n.advertise<rover::Battery> ("/battery", 50);
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState> ("/joint_states", 1);
	ros::Publisher pan_pub = n.advertise<std_msgs::Float64> ("/pan/cur_angle", 1);
	ros::Publisher tilt_pub = n.advertise<std_msgs::Float64> ("/tilt/cur_angle", 1);
	tf::TransformBroadcaster odom_broadcaster;

	// subscribers
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist> (
			"/cmd_vel", 1, cmdVelReceived);
 	ros::Subscriber tilt_angle_sub = n.subscribe<std_msgs::Float64> (
			"/kinect/cur_angle", 1, recvKinectTiltAngle);
  	ros::Subscriber pan_servo_sub = n.subscribe<std_msgs::Float64> (
			"/pan/angle", 1, recvPanServoAngle);
	ros::Subscriber tilt_servo_sub = n.subscribe<std_msgs::Float64> (
			"/tilt/angle", 1, recvTiltServoAngle);

	// open serial port
	if (bot->openSerialPort() == 0) {
		// ROS_INFO("Connected to rover.");
	} else {
		ROS_FATAL("Could not connect to rover.");
		ROS_BREAK();
	}

	usleep(5e6);

	ros::Time current_time, last_time;

	long packetNum = 0;

	ros::Rate r(10.0);

	sensor_msgs::JointState joint_state;

	bot->setPID(lP, lI, lD, rP, rI, rD);
    	bot->setConversionFactors(le, re);

	// main processing loop
	while (n.ok()) {
		current_time = ros::Time::now();

		if (bot->newPacket) {
			bot->newPacket = false;
			// generate quaternion from odometry yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
					bot->m_odometry_yaw);

			// create a tf message
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "/odom";
			odom_trans.child_frame_id = "base_footprint";

			odom_trans.transform.translation.x = bot->m_odometry_x;
			odom_trans.transform.translation.y = bot->m_odometry_y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			// create a odometry msg
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "/odom";

			// set the position
			odom.pose.pose.position.x = bot->m_odometry_x;
			odom.pose.pose.position.y = bot->m_odometry_y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			// set the velocity
			odom.child_frame_id = "base_footprint";
			odom.twist.twist.linear.x = bot->m_velocity_x;
			odom.twist.twist.linear.y = bot->m_velocity_y;
			odom.twist.twist.angular.z = bot->m_velocity_yaw;

			// set the covariance
			double covariance[36] = {1e-3, 0, 0, 0, 0, 0,
						 0, 1e-3, 0, 0, 0, 0,
						 0, 0, 1e6, 0, 0, 0,
						 0, 0, 0, 1e6, 0, 0,
						 0, 0, 0, 0, 1e6, 0,
						 0, 0, 0, 0, 0, 1e3};

			for (int i = 0; i < 36; i++) {
				odom.pose.covariance[i] = covariance[i];
				odom.twist.covariance[i] = covariance[i];
			}

			// publish the message
			odom_pub.publish(odom);

			// create a quaternion for the gyro
			geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(
					bot->m_gyro_yaw);
			
			// create a sensor msg for the gyro
			sensor_msgs::Imu imu_msg;
			imu_msg.header.stamp = current_time;
			imu_msg.header.frame_id = "base_footprint";
			
			imu_msg.orientation = imu_quat;

			imu_msg.angular_velocity.x = 0.0;
			imu_msg.angular_velocity.y = 0.0;
			imu_msg.angular_velocity.z = bot->m_gyro_yawrate;

			imu_msg.linear_acceleration_covariance[0] = -1; // no accelerometers, so just set covariance to -1

			imu_msg.linear_acceleration.x = 0.0;
			imu_msg.linear_acceleration.y = 0.0;
			imu_msg.linear_acceleration.z = 0.0;

			// send the message
			imu_pub.publish(imu_msg);

			// prepare a battery state message
			rover::Battery batt_msg;
			batt_msg.header.stamp = current_time;
			batt_msg.header.frame_id = "base_footprint";
			batt_msg.voltage = bot->m_battery_voltage;
			batt_msg.current = bot->m_battery_current;
			
			// send the message
			batt_pub.publish(batt_msg);

			// prepare an encoder message
			rover::Encoder enc_msg;
			enc_msg.header.stamp = current_time;
			enc_msg.header.frame_id = "base_footprint";
			enc_msg.left = bot->m_velocity_left;
			enc_msg.right = bot->m_velocity_right;

			// send the message
			enc_pub.publish(enc_msg);

			// prepare a joint state message
			joint_state.header.stamp = current_time;

			joint_state.name.resize(7);
			joint_state.position.resize(7);

			joint_state.name[0]="kinect_joint";
			joint_state.position[0]=-tilt_angle;

			joint_state.name[1]="front_left_wheel_joint";
			joint_state.position[1]=bot->m_velocity_left / 0.027051;
			
			joint_state.name[2]="front_right_wheel_joint";
			joint_state.position[2]=bot->m_velocity_right / 0.027051;

			joint_state.name[3]="back_left_wheel_joint";
			joint_state.position[3]=bot->m_velocity_left / 0.027051;

			joint_state.name[4]="back_right_wheel_joint";
			joint_state.position[4]=bot->m_velocity_right / 0.027051;

			joint_state.name[5]="pan_servo_joint";
			joint_state.position[5]=-(bot->m_pan_angle);

			joint_state.name[6]="tilt_servo_joint";
			joint_state.position[6]=(bot->m_tilt_angle);

			joint_pub.publish(joint_state);

			// prepare messages for pan/tilt servos
			std_msgs::Float64 pan_msg, tilt_msg;

			pan_msg.data = -(bot->m_pan_angle);
			pan_pub.publish(pan_msg);

			tilt_msg.data = -(bot->m_tilt_angle);
			tilt_pub.publish(tilt_msg);
		}
		
		// spin the node
		ros::spinOnce();
		// wait for next call
		r.sleep();
	}

	// on exit close the serial port
	bot->closeSerialPort();
}

#ifndef TELEOP_ROVER_H_
#define TELEOP_ROVER_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
#include "util.h"

#define KINECT_LIMIT_ANGLE 0.523598776

#ifndef expo
#define expo(x) ((x) < (0)) ? ((-1) * (x) * (x)) : ((x) * (x))
#endif

class TeleopRover {
public:
  TeleopRover();
private:
  void joyCallback(const joy::Joy::ConstPtr& joy);
  void kinectCallback(const std_msgs::Float64::ConstPtr& angle);

  ros::NodeHandle m_n;

  int m_linear, m_angular, m_panservo, m_tiltservo, m_tiltkinect_up, m_tiltkinect_down;

  int m_enable, m_autonomous_on, m_autonomous_off;

  bool m_autoMode;

  double m_kinect_angle;

  double m_limit_speed, m_limit_angular_speed;

  ros::Time lastUpdateTime;

  ros::Publisher m_cmd_vel_pub;
  ros::Publisher m_servo_tilt_pub;
  ros::Publisher m_servo_pan_pub;
  ros::Publisher m_kinect_tilt_pub;

  ros::Subscriber m_kinect_tilt_sub;
  ros::Subscriber m_joy_sub;
};

#endif

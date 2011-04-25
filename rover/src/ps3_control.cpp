#include <ros/ros.h>
#include "TeleopRover.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_rover");
	TeleopRover teleoprover;
	ros::spin();
}

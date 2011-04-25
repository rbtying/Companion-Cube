#include <ros/ros.h>
#include "RoverGui.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "rover_gui");
	
	RoverGui gui;

	gui.loop();

	ros::spin();
}

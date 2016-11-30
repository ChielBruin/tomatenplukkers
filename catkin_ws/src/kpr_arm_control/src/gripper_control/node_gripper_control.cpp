#include "gripper_control.cpp"

#include "ros/ros.h"

using namespace ros;

const std::string NODE_NAME = "Gripper control";

int main(int argc, char **argv) {
	init(argc, argv, "Gripper control");
	ROS_INFO("Started");
	NodeHandle n;

	spin();
	ROS_INFO("Stopped");
	return 0;
}

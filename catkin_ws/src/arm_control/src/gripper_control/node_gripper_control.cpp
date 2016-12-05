#include "gripper_control.cpp"

#include "ros/ros.h"

using namespace ros;

int main(int argc, char **argv) {
	init(argc, argv, "Gripper control");
	ROS_INFO("Gripper control started");
	NodeHandle n;

	spin();
	return 0;
}

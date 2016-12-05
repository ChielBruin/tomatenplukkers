#include "ros/ros.h"

#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

using namespace ros;

bool moveArmTo(geometry_msgs::Pose targetPos) {
	//TODO Code to move arm to cucumber.
	return false;
}

bool startGrip(CucumberContainer cucumber) {
	//TODO Code to start the grip.
	return false;
}

void cut() {
	//TODO Code to cut the cucumber.
}

bool releaseGrip() {
	//TODO Code to release grip.
	return false;
}

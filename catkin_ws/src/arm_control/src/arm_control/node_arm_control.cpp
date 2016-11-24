#include "arm_control.cpp"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "cucumber_msgs/HarvestAction.h"

#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/CucumberContainer.h"

using namespace ros;

bool getCucumber(cucumber_msgs::HarvestAction::Request &msg, cucumber_msgs::HarvestAction::Response &response) {
	bool success = true;

	CucumberContainer target = CucumberContainer(msg.cucumber);
	success = moveArmTo(target.createPose());
	if (!success) {
		response.status = response.MOVE_ERR;
		return success;
	}

	success = startGrip(target);
	if (!success) {
		response.status = response.GRAB_ERR;
		return success;
	}

	cut();

	success = moveArmTo(msg.dropLocation);
	if (!success) {
		response.status = response.MOVE_ERR;
		return success;
	}

	success = releaseGrip();
	if (!success) {
		response.status = response.DROP_ERR;
		return success;
	}

	return true;
}

int main(int argc, char **argv) {
	init(argc, argv, "Arm control");
	ROS_INFO("Arm control started");
	
	NodeHandle n;
	ServiceServer cucumberService = n.advertiseService("target/cucumber", getCucumber);
	
	spin();
	return 0;
}

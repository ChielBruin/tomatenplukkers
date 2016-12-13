#include "ros/ros.h"
#include "arm_control.cpp"
#include "moveit_consts.h"
#include <memory>

#include "cucumber_msgs/HarvestAction.h"
#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

#include <ur_msgs/IOStates.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <moveit_msgs/DisplayRobotState.h>

#include<cmath> // TODO Remove later.

using namespace ros;

const std::string NODE_NAME = "Arm Control";

float getRandFloat(float min, float max) {
	float result = min + static_cast<float>(rand()) /
			static_cast<float>(RAND_MAX/(max-min));
	return result;
}

geometry_msgs::Pose generateValidPose() {
	geometry_msgs::Pose pose;
	float maxDistance = 0.8;
	float x = getRandFloat(0, maxDistance/2);
	pose.position.x = x;
	float y = getRandFloat(0, maxDistance/2);
	pose.position.y = y;
	maxDistance = maxDistance - sqrt(x * x + y * y);
	float z = getRandFloat(2*maxDistance/3, maxDistance);
	pose.position.z = z;
	ROS_INFO("Generated distance: %f", sqrt(x*x + y*y + z*y));
	return pose;
}

bool getCucumber(cucumber_msgs::HarvestAction::Request &msg,
		cucumber_msgs::HarvestAction::Response &response) {
	bool success = true;

	ROS_INFO("Got cucumber");
	
	if (!moveit_consts::move_group_ptr) {
		ROS_ERROR("MoveGroup has been destroyed!");
	}

	CucumberContainer target = CucumberContainer(msg.cucumber);
	ROS_INFO("Sending move to arm");
	// success = moveArmTo(target.createPose());
	success = moveArmTo(generateValidPose());
	if (!success) {
		response.status = response.MOVE_ERR;
		ROS_ERROR("Moving to cucumber has failed!");
		return true;
	}
	ROS_INFO("Move arm done");

	success = startGrip();
	if (!success) {
		response.status = response.GRAB_ERR;
		ROS_ERROR("Grabbing cucumber has failed!");
		return true;
	}
	ROS_INFO("Grip done");

	cut();
	ROS_INFO("Cut done");

	// success = moveArmTo(msg.dropLocation);
	success = moveArmTo(generateValidPose());
	if (!success) {
		response.status = response.MOVE_ERR;
		ROS_ERROR("Moving to drop off has failed!");
		return true;
	}
	ROS_INFO("Move arm back done");

	success = releaseGrip();
	if (!success) {
		response.status = response.DROP_ERR;
		ROS_ERROR("Releasing grip has failed!");
		return true;
	}
	ROS_INFO("Release grip done");

	return true;
}

void setupRos(NodeHandle n) {
	moveit_consts::move_group_ptr = MoveGroupPtr(new moveit::
			planning_interface::MoveGroup(moveit_consts::move_group_name));
	moveit_consts::move_group_ptr->setNumPlanningAttempts(5);
	moveit_consts::planning_scene_interface_ptr = PlanningSceneInterfacePtr(
			new moveit::planning_interface::PlanningSceneInterface());

	ROS_INFO("Reference frame: %s", moveit_consts::move_group_ptr->
			getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", moveit_consts::move_group_ptr->
			getEndEffectorLink().c_str());
}

int main(int argc, char **argv) {
	init(argc, argv, NODE_NAME);
	ROS_INFO("Started");
	NodeHandle n;

	ServiceServer cucumberService = n.advertiseService("target/cucumber",
			getCucumber);
	io_state_publisher = n.advertise<ur_msgs::IOStates>("iostates", 1);

	setupRos(n);

	// At least two threads necessary to receive planning feedback.
	AsyncSpinner spinner(2);
	spinner.start();
	waitForShutdown();

	ROS_INFO("Stopped");
	return 0;
}

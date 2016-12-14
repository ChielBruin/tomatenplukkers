#include "ros/ros.h"
#include "arm_control.cpp"
#include <memory>

#include "cucumber_msgs/HarvestAction.h"
#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include <ur_msgs/SetIO.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace ros;

const std::string NODE_NAME = "Arm Control";
const std::string move_group_name("manipulator");

bool getCucumber(cucumber_msgs::HarvestAction::Request &msg,
		cucumber_msgs::HarvestAction::Response &response) {
	bool success = true;

	ROS_INFO("Got cucumber");

	CucumberContainer target = CucumberContainer(msg.cucumber);
	ROS_INFO("Sending move to arm");
	success = moveArmTo(target.createPose());
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

	success = cut();
	if (!success) {
		response.status = response.CUTT_ERR;
		ROS_ERROR("Cutting has failed!");
		return true;
	}
	ROS_INFO("Cut done");

	success = moveArmTo(msg.dropLocation);
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
	move_group_ptr = MoveGroupPtr(new moveit::
			planning_interface::MoveGroup(move_group_name));
	move_group_ptr->setNumPlanningAttempts(1);
	move_group_ptr->setPlanningTime(0.5);
	planning_scene_interface_ptr = PlanningSceneInterfacePtr(
			new moveit::planning_interface::PlanningSceneInterface());

	ROS_INFO("Reference frame: %s", move_group_ptr->getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", move_group_ptr->getEndEffectorLink().c_str());
}

int main(int argc, char **argv) {
	init(argc, argv, NODE_NAME);
	ROS_INFO("Started");
	NodeHandle n;

	ServiceServer cucumberService = n.advertiseService("target/cucumber", getCucumber);
	io_state_client = n.serviceClient<ur_msgs::SetIO>("set_io");

	setupRos(n);

	// At least two threads necessary to receive planning feedback.
	AsyncSpinner spinner(2);
	spinner.start();
	waitForShutdown();

	ROS_INFO("Stopped");
	return 0;
}

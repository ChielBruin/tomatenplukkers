#include "ros/ros.h"
#include "moveit_consts.h"
#include <memory>

#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

const std::string FRAME_NAME = "endeffector";
const std::string MOVE_GROUP_NAME = "endeffector";
const std::string CONSTRAINT_LINK_NAME = "tool0";

const float PLANNING_TIMEOUT = 1.0;

int seq = 0;

using namespace ros;
/**
 * Moves the arm to a certain pose.
 * 
 * @param [in] plannerManager The instance of the planner manager to use.
 * @param [in] planningScene The instance of the planning scene to use.
 * @param [in] targetPose The target pose at the end of the action.
 * 
 * @return True if the movement succeeded, false otherwise.
 */
bool moveArmTo(geometry_msgs::Pose targetPose) {
	if (!moveit_consts::move_group_ptr) {
		ROS_ERROR("The Move Group does not exist!");
		return false;
	}

	targetPose.orientation.w = 1.0;
	moveit_consts::move_group_ptr->setPoseTarget(targetPose);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = moveit_consts::move_group_ptr->plan(my_plan);

	if (!success) {
		ROS_ERROR("Could not plan a move; move is probably impossible.");
		return false;
	}
	moveit_consts::move_group_ptr->move();

	return success;
}

bool startGrip(CucumberContainer cucumber) {
	return false;
}

void cut() {
	//TODO Code to cut the cucumber.
}

bool releaseGrip() {
	//TODO Code to release grip.
	return false;
}

#include "ros/ros.h"
#include "moveit_consts.h"
#include <memory>

#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

#include <ur_msgs/IOStates.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

using namespace ros;

const uint8_t GRIPPER_PIN = 0x0;
const uint8_t CUTTER_PIN = 0x0;

Publisher io_state_publisher;

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
	ROS_INFO("Attempting to move to %f,%f,%f", targetPose.position.x,
			targetPose.position.y, targetPose.position.z);

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

bool startGrip() {
	ur_msgs::IOStates io_message;
	ur_msgs::Digital out_state;
	out_state.pin = GRIPPER_PIN;
	out_state.state = true;
	io_message.digital_in_states.push_back(out_state);
	io_state_publisher.publish(io_message);
	return true;
}

void cut() {
	ur_msgs::IOStates io_message;
	ur_msgs::Digital out_state;
	out_state.pin = CUTTER_PIN;
	out_state.state = true;
	io_message.digital_in_states.push_back(out_state);
	io_state_publisher.publish(io_message);
}

bool releaseGrip() {
	ur_msgs::IOStates io_message;
	ur_msgs::Digital out_state;
	out_state.pin = GRIPPER_PIN;
	out_state.state = false;
	io_message.digital_in_states.push_back(out_state);
	io_state_publisher.publish(io_message);
	return true;
}

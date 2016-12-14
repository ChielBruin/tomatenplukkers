#include "ros/ros.h"
#include <memory>

#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

#include <ur_msgs/SetIO.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

using namespace ros;

typedef std::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
typedef std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningSceneInterfacePtr;

const uint8_t VACUUM_PIN = 0x1;
const uint8_t GRIPPER_PIN = 0x2;
const uint8_t CUTTER_PIN = 0x4;

const float GRIPPER_CLOSE = 0x1; // Not sure about value
const float GRIPPER_OPEN = 0x0; // Not sure about value
const float CUTTER_CLOSE = 0x1; // Not sure about value
const float CUTTER_OPEN = 0x0; // Not sure about value

ServiceClient io_state_client;
MoveGroupPtr move_group_ptr;
PlanningSceneInterfacePtr planning_scene_interface_ptr;

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
	if (!move_group_ptr) {
		ROS_ERROR("The Move Group does not exist!");
		return false;
	}
	double distance = sqrt(pow(targetPose.position.x,2) +
			pow(targetPose.position.y,2) + pow(targetPose.position.z,2));

	move_group_ptr->setPoseTarget(targetPose);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = move_group_ptr->plan(my_plan);

	if (!success) {
		ROS_ERROR("Could not plan a move; move is probably impossible.");
		if (distance > 0.8) {
			ROS_ERROR("Move failed because location was out of reach.");
		} else if (distance < 0.25) {
			ROS_ERROR("Move failed because location was probably too close to the base");
		}
		return success;
	}
	move_group_ptr->move();

	return success;
}

bool startGrip() {
	ur_msgs::SetIO io_message;
	io_message.request.fun = io_message.request.FUN_SET_DIGITAL_OUT;
	io_message.request.pin = GRIPPER_PIN;
	io_message.request.state = GRIPPER_CLOSE;
	return io_state_client.call(io_message) && io_message.response.success;
}

bool cut() {
	ur_msgs::SetIO io_message_close;
	io_message_close.request.fun = io_message_close.request.FUN_SET_DIGITAL_OUT;
	io_message_close.request.pin = CUTTER_PIN;
	io_message_close.request.state = CUTTER_CLOSE;
	if (!(io_state_client.call(io_message_close) && io_message_close.response.success)) {
		return false;
	}

	//TODO Not sure if a timeout should be here.
	ur_msgs::SetIO io_message_open;
	io_message_open.request.fun = io_message_open.request.FUN_SET_DIGITAL_OUT;
	io_message_open.request.pin = CUTTER_PIN;
	io_message_open.request.state = CUTTER_OPEN;
	return io_state_client.call(io_message_close) && io_message_close.response.success;
}

bool releaseGrip() {
	ur_msgs::SetIO io_message;
	io_message.request.fun = io_message.request.FUN_SET_DIGITAL_OUT;
	io_message.request.pin = GRIPPER_PIN;
	io_message.request.state = GRIPPER_OPEN;
	return io_state_client.call(io_message) && io_message.response.success;
}

#include "ros/ros.h"
#include <memory>

#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

#include <ur_msgs/SetIO.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/MoveItErrorCodes.h>

using namespace ros;

typedef std::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
typedef std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningSceneInterfacePtr;

const uint8_t VACUUM_PIN = 0x1;
const uint8_t GRIPPER_PIN = 0x2;
const uint8_t CUTTER_PIN = 0x4;

const float GRIPPER_CLOSE = 0x0;
const float GRIPPER_OPEN = 0x1;
const float CUTTER_CLOSE = 0x1;
const float CUTTER_OPEN = 0x0;

ServiceClient io_state_client;
MoveGroupPtr move_group_ptr;
PlanningSceneInterfacePtr planning_scene_interface_ptr;

/**
 * Moves the arm to a certain pose.
 * 
 * @param targetPose The target pose at the end of the action.
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
		if (distance > 0.8) {
			ROS_ERROR("Planning failed because location was out of reach.");
		} else if (distance < 0.25) {
			ROS_ERROR("Planning failed because location was probably too close to the base");
		} else {
			ROS_ERROR("Could not plan a move; move is probably impossible.");
		}
		return false;
	}
	moveit_msgs::MoveItErrorCodes errorCode = move_group_ptr->move();
	if (errorCode.val == errorCode.SUCCESS) {
		return true;
	} else {
		ROS_ERROR("Could not move the robot after move plan was obtained.");
		ROS_ERROR("MoveIt! returned error code %d. For the meaning of this"
				"error visit http://docs.ros.org/jade/api/moveit_msgs/html"
				"/msg/MoveItErrorCodes.html", errorCode.val);
		return false;
	}
}

/**
 * Sends the given IO state to the given pin. Automatically uses the
 * SET_DIGITAL_OUT function.
 * 
 * @param pin The pin the IO message should be sent to.
 * @param state The state that should be sent to the pin.
 * 
 * @return True if the IO call succeeded, false otherwise.
 */
bool sendIO(int8_t pin, float state) {
	ur_msgs::SetIO io_message;
	io_message.request.fun = io_message.request.FUN_SET_DIGITAL_OUT;
	io_message.request.pin = pin;
	io_message.request.state = state;
	return io_state_client.call(io_message) && io_message.response.success;
}

/**
 * Sends the necessary IO message to close the gripper.
 * @return True if the IO call succeeded, false otherwise.
 */
bool startGrip() {
	bool success = sendIO(GRIPPER_PIN, GRIPPER_CLOSE);
	//TODO Wait for IO to confirm the gripper has closed.
	// Attempt to close multiple times in slightly different conditions
	// if it failed.
	return success;
}

/**
 * Sends the necessary IO message to do a cut.
 * @return True if the IO call succeeded, false otherwise.
 */
bool cut() {
	if (!sendIO(CUTTER_PIN, CUTTER_CLOSE)) {
		return false;
	}
	//TODO Add wait for IO to confirm the cutter has closed.
	return sendIO(CUTTER_PIN, CUTTER_OPEN);
}

/**
 * Sends the necessary IO message to open the gripper.
 * @return True if the IO call succeeded, false otherwise.
 */
bool releaseGrip() {
	return sendIO(GRIPPER_PIN, GRIPPER_OPEN);
}

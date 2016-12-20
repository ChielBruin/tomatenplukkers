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

/** Definitions related to the output pins */
const uint8_t VACUUM_PIN = 0x1;
const uint8_t GRIPPER_PIN = 0x2;
const uint8_t CUTTER_PIN = 0x4;

const float GRIPPER_CLOSE = 0x0;
const float GRIPPER_OPEN = 0x1;
const float CUTTER_CLOSE = 0x1;
const float CUTTER_OPEN = 0x0;
const float VACUUM_ON = 0X1;
const float VACUUM_OFF = 0X0;

/** Definitions related to the input pins */
const uint8_t PRESSURE_PIN = 0x1; 
const uint8_t CUT_CLOSED_PIN = 0x2; 
const uint8_t CUT_OPEN_PIN = 0x3; 
//TODO  add definition for the sensor related to the gripper

bool pinstates[] = {false, false, true};
#define PRESSURE_STATE 0
#define CUT_CLOSED_STATE 1
#define CUT_OPEN_STATE 2

const bool expectedValues[] = {true, false, true, true};
#define EXPECT_VAC 0
#define EXPECT_NOVAC 1
#define EXPECT_CUT_OPEN 2
#define EXPECT_CUT_CLOSED 3

const uint8_t nrOfAttemps = 10;


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
 * Function which checks whether the previously changed output has the desired effect.
 * It makes sure that the value checked is indeed the response of the new sensor and 
 * not the old response, since it is internally updated at f = 10 Hz. 
 * 
 * @param sensor The sensor which output needs to be read.
 * @param expect The value expected from the sensor.
 * 
 * @return Passes the boolean value received from the sensors.
 */
bool readDigInput(int sensor, bool expect) {
	Time time = Time::now();
	while (Time::now() <= (time + Duration(0.1))) {
		if (pinstates[sensor] == expect) {
			ROS_DEBUG("The command has been successfully performed, according to the input");
			return true;
			break;
		}
		else {
			ROS_DEBUG("The command has not been successfully performed, according to the input");
		}
	}
	return false;
}

/**
 * Sends the necessary IO message to close the gripper.
 * @return True if the IO call succeeded, false otherwise.
 */
bool startGrip() {
	if (!sendIO(GRIPPER_PIN, GRIPPER_CLOSE)) {
		return false;
	}
	ROS_DEBUG("Open command has been send to the gripper");
	
	//TODO Wait for IO to confirm the gripper has closed.
	// Attempt to close multiple times in slightly different conditions
	// if it failed.
	return true;
}

/**
 * Sends the necessary IO message to do a cut.
 * @return True if the IO calls succeeded and the sensors confirm the expected result, false otherwise.
 */
bool cut() {
	if (!sendIO(CUTTER_PIN, CUTTER_CLOSE)) {
		return false;
	}
	ROS_DEBUG("Close command has been send to the cutter");
	
	bool success = false;
	for (int tries = 0; tries <= nrOfAttemps; tries++) {
		success = readDigInput(CUT_CLOSED_STATE, expectedValues[EXPECT_CUT_CLOSED]);
		if (success) {
			break; 
		}
			ROS_DEBUG("Cutter is not open according to IO and a new attempt would need to be made");
			//TODO Add new commands due to failed attempt.
	}
	if (!success) {
		return false;
	}
		
	if (!sendIO(CUTTER_PIN, CUTTER_OPEN)) {
		return false;
	}
	ROS_DEBUG("Open command has been send to the cutter");

	for (int tries = 0; tries <= nrOfAttemps; tries++) {
		if (readDigInput(CUT_OPEN_STATE, expectedValues[EXPECT_CUT_OPEN])) {
			return true; 
		}
			ROS_DEBUG("Cutter is not open according to IO and a new attempt would need to be made");
			//TODO Add new command due to failed attempt.
	}
	if (!success) {
		return false;
	}
}

/**
 * Sends the necessary IO message to open the gripper.
 * @return True if the IO call succeeded, false otherwise.
 */
bool releaseGrip() {
	if (!sendIO(GRIPPER_PIN, GRIPPER_OPEN)) {
		return false;
	}
	//TODO Wait for IO to confirm the gripper has closed.
	return true;
}

/**
 * Sends the necessary IO message to enable the vacuum suction.
 * @return True if the IO call succeeded and the sensor confirms the expected result, false otherwise.
 */
bool startVacuum() {
	if (!sendIO(VACUUM_PIN, VACUUM_ON)) {
		return false;
	}

	for (int tries = 0; tries <= nrOfAttemps; tries++) {
		if (readDigInput(PRESSURE_STATE, expectedValues[EXPECT_VAC])) {
			return true;
		}
			ROS_DEBUG("Suction is not correct according to IO and a new attempt would need to be made");
			//TODO Add new commands due to failed attempt.
	}
	return false;
}

/**
 * Sends the necessary IO message to disable the vacuum suction.
 * @return True if the IO call succeeded and the sensor confirms the expected result, false otherwise.
 */
bool stopVacuum() {
	if (!sendIO(VACUUM_PIN, VACUUM_OFF)) {
		return false;
	}
	
	if (!readDigInput(PRESSURE_STATE, expectedValues[EXPECT_NOVAC])) {
		ROS_WARN("The vacuum should have been 'turned off', but the sensor still measured a vacuum.");
		return false;
	}
	//TODO Could require mutliple attempts, if the sensor does not confirm with the expected value.
	return true;
}

#include "ros/ros.h"
#include "arm_control.cpp"
#include <memory>

#include "cucumber_msgs/HarvestAction.h"
#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <ur_msgs/Digital.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace ros;

const std::string NODE_NAME = "Arm Control";
const std::string move_group_name("manipulator");

/**
 * Attempts to pick a cucumber. This function does the following things:
 * - It attempts to move the arm to the location of the cucumber.
 * - It attempts to grip the cucumber.
 * - It attempts to attach to the cucumber via a suction cap.
 * - It attempts to cut the stem of the cucumber.
 * - It attempts to move the arm to the drop-off location.
 * - It attempts to let go of the cucumber.
 * 
 * @param msg The Request to pick a certain cucumber.
 * @param response The Response that will be sent to the client.
 * 
 * @return True if the service didn't encounter critical errors, false otherwise.
 */
bool getCucumber(cucumber_msgs::HarvestAction::Request &msg,
		cucumber_msgs::HarvestAction::Response &response) {
	bool success = true;

	ROS_DEBUG("Received cucumber from core.");

	CucumberContainer target = CucumberContainer(msg.cucumber);
	ROS_DEBUG("Sending move to arm");
	success = moveArmTo(target.createPose());
	if (!success) {
		response.status = response.MOVE_ERR;
		ROS_ERROR("Moving to cucumber has failed!");
		return true;
	}
	ROS_DEBUG("Move arm done.");

	success = startGrip();
	if (!success) {
		response.status = response.GRAB_ERR;
		ROS_ERROR("Grabbing cucumber has failed!");
		return true;
	}
	ROS_DEBUG("Grip done.");

	success = startVacuum();
	if (!success) {
		response.status = response.VACU_ERR;
		ROS_ERROR("Generating a vacuum has failed!");
		return true;
	}

	success = cut();
	if (!success) {
		response.status = response.CUTT_ERR;
		ROS_ERROR("Cutting has failed!");
		return true;
	}
	ROS_DEBUG("Cut done.");

	success = moveArmTo(msg.dropLocation);
	if (!success) {
		response.status = response.MOVE_ERR;
		ROS_ERROR("Moving to drop off has failed!");
		return true;
	}
	ROS_DEBUG("Move arm back done.");

	success = stopVacuum();
	if (!success) {
		response.status = response.NOVAC_ERR;
		ROS_ERROR("Stopping the vacuum has failed!");
		return true;
	}

	success = releaseGrip();
	if (!success) {
		response.status = response.DROP_ERR;
		ROS_ERROR("Releasing grip has failed!");
		return true;
	}
	ROS_DEBUG("Release grip done.");
	ROS_INFO("Successfully harvested a cucumber.");

	return true;
}

/**
 * Sets up the moveIt environment.
 * 
 * @param n The node handle used to communicate with the master.
 */
void setupMoveIt(NodeHandle n) {
	move_group_ptr = MoveGroupPtr(new moveit::
			planning_interface::MoveGroup(move_group_name));
	move_group_ptr->setNumPlanningAttempts(1);
	move_group_ptr->setPlanningTime(0.5);
	planning_scene_interface_ptr = PlanningSceneInterfacePtr(
			new moveit::planning_interface::PlanningSceneInterface());

	ROS_INFO("Reference frame: %s", move_group_ptr->getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", move_group_ptr->getEndEffectorLink().c_str());
}

/**
 * Calls the latest state of all IO-ports and the relevant inputs are passed to the vector pinstates.
 */
void getIOStates(const ur_msgs::IOStates msg) {
	std::vector<ur_msgs::Digital> digitalPins = msg.digital_in_states;
	pinstates[PRESSURE_STATE] = digitalPins[PRESSURE_PIN].state;
	pinstates[CUT_CLOSED_STATE] = digitalPins[CUT_CLOSED_PIN].state;
	pinstates[CUT_OPEN_STATE] = digitalPins[CUT_OPEN_PIN].state;
}

/**
 * Starts the arm control node. It does not require any command line parameters.
 * 
 * @param argc The amount of parameters.
 * @param argv The parameters.
 * 
 * @return The status code of the execution.
 */
int main(int argc, char **argv) {
	init(argc, argv, NODE_NAME);
	ROS_INFO("Started");
	NodeHandle n;

	ServiceServer cucumberService = n.advertiseService("target/cucumber", getCucumber);
	io_state_client = n.serviceClient<ur_msgs::SetIO>("set_io");
	
	Subscriber sub_io_states = n.subscribe("io_states", 1, getIOStates);

	setupMoveIt(n);

	// At least two threads necessary to receive planning feedback.
	AsyncSpinner spinner(2);
	spinner.start();
	waitForShutdown();

	ROS_INFO("Stopped");
	return 0;
}

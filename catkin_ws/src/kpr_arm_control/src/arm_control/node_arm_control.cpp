#include "ros/ros.h"
#include "arm_control.cpp"
#include <memory>

#include "cucumber_msgs/HarvestAction.h"
#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/CucumberContainer.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ur_msgs/SetIO.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace ros;

const std::string NODE_NAME = "Arm Control";
const std::string move_group_name("manipulator");

Publisher co_publisher;
Publisher aco_publisher;

/**
 * Attempts to pick a cucumber. This function does the following things:
 * - It attempts to move the arm to the location of the cucumber.
 * - It attempts to grip the cucumber.
 * - It attempts to cut the stem of the cucumber.
 * - It attempts to move the arm to the drop-off location.
 * - It attempts to let the cucumber go.
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

	//TODO Attempt to attach the suction cup to the cucumber.

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
 * Adds the table the arm stands on to the planning scene, to prevent the
 * arm from breaking itself and the end effector. Also adds a backpanel
 * because the arm won't be able to navigate too far backwards because
 * another row of cucumber plants will be there, which won't be visible.
 */
void addTable() {
	// Offset because the base continues slightly below its link.
	const float HEIGHT_OFFSET = 0.004;

	moveit_msgs::AttachedCollisionObject table;
	table.link_name = "world";
	table.object.header.frame_id = "table_attach";
	table.object.id = "table";

	geometry_msgs::Pose tablePose;
	tablePose.position.z = -0.1/2 - HEIGHT_OFFSET;
	tablePose.orientation.w = 1.0;

	shape_msgs::SolidPrimitive tableBox;
	tableBox.type = tableBox.BOX;
	tableBox.dimensions.resize(3);
	tableBox.dimensions[tableBox.BOX_X] = 1.5;
	tableBox.dimensions[tableBox.BOX_Y] = 1;
	tableBox.dimensions[tableBox.BOX_Z] = 0.1;

	table.object.primitives.push_back(tableBox);
	table.object.primitive_poses.push_back(tablePose);

	geometry_msgs::Pose backpanelPose;
	backpanelPose.position.y = -0.3;
	backpanelPose.position.z = 0.5 - HEIGHT_OFFSET;
	backpanelPose.orientation.w = 1.0;

	shape_msgs::SolidPrimitive backpanelBox;
	backpanelBox.type = backpanelBox.BOX;
	backpanelBox.dimensions.resize(3);
	backpanelBox.dimensions[backpanelBox.BOX_X] = 1.5;
	backpanelBox.dimensions[backpanelBox.BOX_Y] = 0.1;
	backpanelBox.dimensions[backpanelBox.BOX_Z] = 1;

	table.object.primitives.push_back(backpanelBox);
	table.object.primitive_poses.push_back(backpanelPose);
	table.object.operation = table.object.ADD;

	aco_publisher.publish(table);
}

/**
 * Adds the temporary end effector, to prevent the arm from breaking the
 * real end effector and to be able to properly align with the cucumber.
 */
void addEndEffector() {
	moveit_msgs::AttachedCollisionObject endEffector;
	endEffector.link_name = "ee_link";
	endEffector.object.header.frame_id = "end_effector_attach";
	endEffector.object.id = "end_effector";

	geometry_msgs::Pose eePose;
	// Translates the end effector to the end of the arm.
	// Should not be necessary but currently the built-in translation breaks.
	//TODO Make sure the built-in translation is used instead of this work around.
	eePose.position.x = 0.81725;
	eePose.position.y = 0.19145;
	eePose.position.z = -0.005491;

	eePose.position.y += 0.16/2;
	eePose.orientation.w = 1.0;

	shape_msgs::SolidPrimitive eeBox;
	eeBox.type = eeBox.BOX;
	eeBox.dimensions.resize(3);
	eeBox.dimensions[eeBox.BOX_X] = 0.075;
	eeBox.dimensions[eeBox.BOX_Y] = 0.16;
	eeBox.dimensions[eeBox.BOX_Z] = 0.075;

	endEffector.object.primitives.push_back(eeBox);
	endEffector.object.primitive_poses.push_back(eePose);
	endEffector.object.operation = endEffector.object.ADD;

	aco_publisher.publish(endEffector);
}

/**
 * Adds the objects in the scene. (table and end effector).
 */
void addSceneObjects() {
	addTable();
	addEndEffector();
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

	co_publisher = n.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	aco_publisher = n.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	addSceneObjects();
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

	setupMoveIt(n);

	// At least two threads necessary to receive planning feedback.
	AsyncSpinner spinner(2);
	spinner.start();
	waitForShutdown();

	ROS_INFO("Stopped");
	return 0;
}

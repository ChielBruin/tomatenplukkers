#include "ros/ros.h"

#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

const std::string FRAME_NAME = "gripper";
const std::string MOVE_GROUP_NAME = "gripper";
const std::string CONSTRAINT_LINK_NAME = "?";

using namespace ros;

/**
 * Moves the arm to a certain pose.
 * 
 * @param [in] planningInstance The instance of the planner manager to use.
 * @param [in] scene The instance of the planning scene to use.
 * @param [in] targetPose The target pose at the end of the action.
 * 
 * @return True if the movement succeeded, false otherwise.
 */
bool moveArmTo(planning_interface::PlannerManagerPtr planningInstance, planning_scene::PlanningScenePtr scene, geometry_msgs::Pose targetPose) {
	// Create messages for moveit.
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	// Create PoseStamped.
	geometry_msgs::PoseStamped poseMsg;
	poseMsg.pose = targetPose;
	poseMsg.header.frame_id = FRAME_NAME;

	// Precision tolerance of the resulting pose. Fine tune if necessary.
	double tolerancePose = 0.01;
	double toleranceAngle = 0.01;

	// Construct movement constraints.
	req.group_name = MOVE_GROUP_NAME;
	moveit_msgs::Constraints poseGoal = kinematic_constraints::constructGoalConstraints(CONSTRAINT_LINK_NAME, poseMsg, tolerancePose, toleranceAngle);
	req.goal_constraints.push_back(poseGoal);

	// Do the actual planning.
	planning_interface::PlanningContextPtr context = planningInstance->getPlanningContext(scene, req, res.error_code_);
	context->solve(res);
	if (res.error_code_.val != res.error_code_.SUCCESS) {
		ROS_ERROR("Could not compute plan successfully");
		return false;
	}
	return true;
}

bool startGrip(CucumberContainer cucumber) {
	//TODO Code to start the grip.
	return false;
}

void cut() {
	//TODO Code to cut the cucumber.
}

bool releaseGrip() {
	//TODO Code to release grip.
	return false;
}

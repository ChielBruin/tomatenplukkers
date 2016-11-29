#include "arm_control.cpp"
#include "ros/ros.h"

#include "cucumber_msgs/HarvestAction.h"
#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/CucumberContainer.h"
#include "geometry_msgs/Pose.h"

#include "pluginlib/class_loader.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

using namespace ros;

planning_scene::PlanningScenePtr scene;
planning_interface::PlannerManagerPtr planner;

bool getCucumber(cucumber_msgs::HarvestAction::Request &msg, cucumber_msgs::HarvestAction::Response &response) {
	bool success = true;

	CucumberContainer target = CucumberContainer(msg.cucumber);
	success = moveArmTo(planner, scene, target.createPose());
	if (!success) {
		response.status = response.MOVE_ERR;
		return success;
	}

	success = startGrip(target);
	if (!success) {
		response.status = response.GRAB_ERR;
		return success;
	}

	cut();

	success = moveArmTo(planner, scene, msg.dropLocation);
	if (!success) {
		response.status = response.MOVE_ERR;
		return success;
	}

	success = releaseGrip();
	if (!success) {
		response.status = response.DROP_ERR;
		return success;
	}

	return true;
}

void setupRos(NodeHandle n) {
	robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
	robot_model::RobotModelPtr robotModel = robotModelLoader.getModel();
	
	planning_scene::PlanningScenePtr planningScene(new planning_scene::PlanningScene(robotModel));
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > plannerPluginLoader;
	planning_interface::PlannerManagerPtr plannerInstance;
	std::string plannerPluginName;
	scene = planningScene;
	planner = plannerInstance;
	
	if (!n.getParam("planning_plugin", plannerPluginName)) {
		ROS_FATAL_STREAM("Could not find planner plugin name");
	}
	try {
		plannerPluginLoader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException& e) {
		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << e.what());
	}
	try {
		plannerInstance.reset(plannerPluginLoader->createUnmanagedInstance(plannerPluginName));
		if (!plannerInstance->initialize(robotModel, n.getNamespace())) {
			ROS_FATAL_STREAM("Could not initialize planner instance");
		}
		ROS_INFO_STREAM("Using planning interface '" << plannerInstance->getDescription() << "'");
	} catch (pluginlib::PluginlibException& e) {
		const std::vector<std::string> &classes = plannerPluginLoader->getDeclaredClasses();
		std::stringstream sstream;
		for (std::size_t i=0; i < classes.size(); i++) {
			sstream << classes[i] << " ";
		}
		ROS_ERROR_STREAM("Exception while loading planner '" << plannerPluginName << ":" << e.what() << std::endl << "Available plugins: " << sstream.str());
	}
}

int main(int argc, char **argv) {
	init(argc, argv, "Arm control");
	ROS_INFO("Arm control started");
	
	NodeHandle n;
	ServiceServer cucumberService = n.advertiseService("target/cucumber", getCucumber);
	
	setupRos(n);
	
	spin();
	return 0;
}

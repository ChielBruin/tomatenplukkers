#include "core.cpp"

#include "ros/ros.h"
#include <string>
#include "geometry_msgs/Pose.h"
#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/HarvestAction.h"
#include "cucumber_msgs/CucumberContainer.h"
#include "kpr_interface/GetSettings.h"
#include "kpr_interface/SetSetting.h"
#include "diagnostic_msgs/KeyValue.h"

#define CRATE_POSITION geometry_msgs::Pose()

using namespace ros;
using namespace std;

/**
 * Callback used for receiving new cucumbers.
 */
void cucumberCallback(const cucumber_msgs::Cucumber msg) {
	CucumberContainer c = CucumberContainer(msg);
	push_back(c);
}

/**
 * Callback for receiving updates of the settings.
 */
void settingsCallback(const kpr_interface::SetSetting msg) {
	map<string, string> settings = map<string, string>();
	for (int i = 0; i < msg.size; i++) {
		diagnostic_msgs::KeyValue kval = msg.settings[i];
		settings.insert(pair<string, string>(kval.key,kval.value));
	}
}

/**
 * Get a forced update of the settings from the settings manager.
 * Note: the returned data must be passed to setSettings() before it can be used.
 * @param nh The nodehandle of this nodes instance.
 */
map<string, string> getSettings(NodeHandle nh) {
	ros::ServiceClient client = nh.serviceClient<kpr_interface::GetSettings>("/settings/getAll");
	kpr_interface::GetSettings srv;
	if (client.call(srv)) {
		map<string, string> settings = map<string, string>();
		for (int i = 0; i < srv.response.size; i++) {
			diagnostic_msgs::KeyValue kval = srv.response.settings[i];
			settings.insert(pair<string, string>(kval.key,kval.value));
		}
		return settings;
	} else {
		ROS_ERROR("Failed to call service getSettings");
		return map<string, string> ();
	}
}

int main(int argc, char **argv) {
	init(argc, argv, "Core");
	ROS_INFO("Started");

	NodeHandle n;
	ServiceClient arm_controller = n.serviceClient<cucumber_msgs::HarvestAction>("target/srv");
	Publisher target_pub = n.advertise<cucumber_msgs::Cucumber>("target", 2);
	Subscriber image_sub = n.subscribe("stereo/cucumber", 10, cucumberCallback);
	Subscriber settings_sub = n.subscribe("settings/update", 10, settingsCallback);
	
	setSettings(getSettings(n));

	Rate loop_rate(10.f);	// 10Hz
	
	while(ros::ok()) {
		spinOnce();
		if (!hasNext()) {
			loop_rate.sleep();
			continue;
		}
		
		CucumberContainer c = pop();
		cucumber_msgs::Cucumber msg = c.toMessage();
		target_pub.publish(msg);
		cucumber_msgs::HarvestAction srv;
		srv.request.cucumber = msg;
		srv.request.dropLocation = CRATE_POSITION;
		if (!arm_controller.call(srv) || srv.response.status != cucumber_msgs::HarvestAction::Response::OK) {
				//TODO: ERROR handling
		}
	}
	
	ROS_INFO("Stopped");	
	return 0;
}

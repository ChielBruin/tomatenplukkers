#include "core.cpp"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "cucumber_msgs/Cucumber.h"
#include "cucumber_msgs/HarvestAction.h"
#include "cucumber_msgs/CucumberContainer.h"

#define CRATE_POSITION geometry_msgs::Pose()

using namespace ros;

void cucumberCallback(const cucumber_msgs::Cucumber msg) {
	CucumberContainer c = CucumberContainer(msg);
	push_back(c);
}

int main(int argc, char **argv) {
	init(argc, argv, "Core");
	ROS_INFO("Core started");

	NodeHandle n;
	ServiceClient arm_controller = n.serviceClient<cucumber_msgs::HarvestAction>("target/cucumber");
	Subscriber image_sub = n.subscribe("stereo/cucumber", 10, cucumberCallback);

	while(ros::ok()) {
		spinOnce();
		
		if (!hasNext()) continue;
		
		CucumberContainer c = pop();
		
		cucumber_msgs::HarvestAction srv;
		srv.request.cucumber = c.toMessage();
		srv.request.dropLocation = CRATE_POSITION;
		if (!arm_controller.call(srv) || srv.response.status != cucumber_msgs::HarvestAction::Response::OK) {
				//TODO: ERROR handling
		}
	}
	
	return 0;
}

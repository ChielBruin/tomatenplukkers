#include "IdCB.cpp"

#include "ros/ros.h"
#include "cucumber_msgs/Cucumber.h"
#include "stereo_msgs/DisparityImage.h"

using namespace ros;

Publisher cucumber_pub;

void left_imageCallback(const cucumber_msgs::Cucumber& c) {
	CucumberContainer res = to3D(c, CAM_LEFT, getDisparityImage(c.header.stamp));
	cucumber_pub.publish(res.toMessage());
}

void right_imageCallback(const cucumber_msgs::Cucumber& c) {
	CucumberContainer res = to3D(c, CAM_RIGHT, getDisparityImage(c.header.stamp));
	cucumber_pub.publish(res.toMessage());
}

void disparityCallback(const stereo_msgs::DisparityImage& msg) {
	setDisparityImage(msg);
}

int main(int argc, char **argv) {
	init(argc, argv, "IdCB");
	ROS_INFO("IdCB started");

	NodeHandle n;
	cucumber_pub = n.advertise<cucumber_msgs::Cucumber>("stereo/cucumber", 20);
	Subscriber image_left_sub = n.subscribe("left/cucumber", 1000, left_imageCallback);
	Subscriber image_right_sub = n.subscribe("right/cucumber", 1000, right_imageCallback);
	Subscriber disparity_sub = n.subscribe("/points2", 1000, disparityCallback);

	spin();
	return 0;
}

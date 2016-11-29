#include "IdCB.cpp"

#include "ros/ros.h"
#include "cucumber_msgs/Cucumber.h"
#include "sensor_msgs/PointCloud2.h"

using namespace ros;

Publisher cucumber_pub;

void left_imageCallback(const cucumber_msgs::Cucumber& c) {
	CucumberContainer res = to3D(c, CAM_LEFT, getPointCloud(c.header.stamp));
	cucumber_pub.publish(res.toMessage());
}

void right_imageCallback(const cucumber_msgs::Cucumber& c) {
	CucumberContainer res = to3D(c, CAM_RIGHT, getPointCloud(c.header.stamp));
	cucumber_pub.publish(res.toMessage());
}

void pointCloudCallback(const sensor_msgs::PointCloud2& msg) {
	setPointCloud(msg);
}

int main(int argc, char **argv) {
	init(argc, argv, "IdCB");
	ROS_INFO("Started");

	NodeHandle n;
	cucumber_pub = n.advertise<cucumber_msgs::Cucumber>("stereo/cucumber", 20);
	Subscriber image_left_sub = n.subscribe("left/cucumber", 1000, left_imageCallback);
	Subscriber image_right_sub = n.subscribe("right/cucumber", 1000, right_imageCallback);
	Subscriber pointcloud_sub = n.subscribe("depth/points", 1000, pointCloudCallback);

	spin();
	ROS_INFO("Stopped");
	return 0;
}

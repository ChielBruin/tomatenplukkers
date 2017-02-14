#include "cVis.cpp"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cucumber_msgs/Cucumber.h"
#include "ros_faster_rcnn/Detection.h"
#include "ros_faster_rcnn/DetectionArray.h"

using namespace ros;

const std::string SIDE = "left";
Publisher cucumber_pub;
Publisher rcnn_pub;
 
/**
 * Callback used when images are received.
 * Passes the image to the processing node.
 */
void imageCallback(const sensor_msgs::Image& image) {
	rcnn_pub.publish(image);
}

/**
 * Callback for receiving detections from the detection node
 * Parses the detected cucumbers to a cucumber message and publishes it.
 */
void detectionCallback(const ros_faster_rcnn::DetectionArray& msg) {
	std::vector<ros_faster_rcnn::Detection> cucumbers;
	std::vector<ros_faster_rcnn::Detection> tops;

	for (int i = 0; i < msg.size; i++) {
		ros_faster_rcnn::Detection d = msg.data[i];
		if (! checkDetection(d)) continue;
		if (d.object_class.compare("cucumber") == 0) {
			cucumbers.push_back(d);
		} else {
			tops.push_back(d);
		}
	}

	for (ros_faster_rcnn::Detection d : cucumbers) {
		cucumber_msgs::Cucumber res = processDetection(d, tops).toMessage();
		res.header = msg.header;
		cucumber_pub.publish(res);
	}
}

/**
 * Main method for the cVis node.
 * Connects the publisher and subscriber and starts spinning.
 */
int main(int argc, char **argv) {
	init(argc, argv, "cVis_" + SIDE);
	NodeHandle n;
	ROS_INFO("Started");

	cucumber_pub = n.advertise<cucumber_msgs::Cucumber>("/left/cucumber", 20);
	Subscriber image_sub = n.subscribe("/camera/left/image_rect_color", 1000, imageCallback);
	
	rcnn_pub = n.advertise<sensor_msgs::Image>("/rcnn/image_raw", 20);
	Subscriber detector_sub = n.subscribe("/rcnn/res/array", 1000, detectionCallback);
	spin();
	
	ROS_INFO("Stopped");
	return 0;
}

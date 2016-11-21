#include "cVis.cpp"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cucumber_msgs/Cucumber.h"

using namespace ros;

const std::string SIDE = "left";
Publisher cucumber_pub;
 
/**
 * Callback used when images are received.
 * Passes the image to the processing algorithm and publishes a message for each cucumber detected.
 */
void imageCallback(const sensor_msgs::Image& image) {
	std::vector<CucumberContainer> res = processImage(image);
	for (auto &c : res) {
		cucumber_msgs::Cucumber msg = c.toMessage();
		 msg.header = image.header;
		 msg.dimension = cucumber_msgs::Cucumber::DIM_2D;
		cucumber_pub.publish(msg);
	} 
}

/**
 * Main method for the cVis node.
 * Connects the publisher and subscriber and starts spinning.
 */
int main(int argc, char **argv) {
	init(argc, argv, "cVis_" + SIDE);
	ROS_INFO("cVis_%s started", SIDE.c_str());

	NodeHandle n;
	cucumber_pub = n.advertise<cucumber_msgs::Cucumber>(SIDE + "/cucumber", 20);
	Subscriber image_sub = n.subscribe(SIDE + "/image_raw", 1000, imageCallback);

	spin();
	return 0;
}

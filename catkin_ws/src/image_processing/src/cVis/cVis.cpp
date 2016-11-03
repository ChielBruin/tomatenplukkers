#include "cucumber_msgs/CucumberContainer.h"
#include "ros_faster_rcnn/Detection.h"

#include "sensor_msgs/Image.h"

#define HEIGHT_OFFSET 0

/**
 * Check if the detected object is valid.
 * This is done by validation of the object class and certainty of the detection.
 */
bool checkDetection(ros_faster_rcnn::Detection det) {
	bool name = det.object.compare("Cucumber") == 0;
	bool p = det.p > .8;
	return name && p;
}

/**
 * Calculates the curvature of the detected cucumber.
 */
float calculateCurvature(float width, float height) {
	//TODO
	return 0;
}

/**
 * Processes the detection and produces a CucumberContainer object.
 */
CucumberContainer processDetection(ros_faster_rcnn::Detection det) {	
	float width = det.width;
	float height = det.height;
	
	float x = det.x + (width * .5);
	float y = det.y + (height * HEIGHT_OFFSET);
	
	float curvature = calculateCurvature(det.width, det.height);
	
	CucumberContainer res = CucumberContainer(x, y, width, height, curvature);
	return res;
}

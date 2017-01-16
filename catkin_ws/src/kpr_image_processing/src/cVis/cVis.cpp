#include "cucumber_msgs/CucumberContainer.h"
#include "ros_faster_rcnn/Detection.h"

#include "sensor_msgs/Image.h"
#include <math.h>
#include <limits>

#define MAX_DIST 200
#define DETECTION_TRESHOLD 0.8

/**
 * Check if the detected object is valid.
 * This is done by validation of the object class and certainty of the detection.
 */
bool checkDetection(ros_faster_rcnn::Detection det) {
	bool name = det.object_class.compare("Cucumber") == 0 || det.object_class.compare("CucumberTop") == 0;
	bool p = det.p > DETECTION_TRESHOLD;
	return name && p;
}

/**
 * Calculates the curvature of the detected cucumber.
 */
float calculateCurvature(float topWidth, float cucumberWidth, float cucumberHeight) {
	float delta = std::max(0.0f, cucumberWidth - topWidth);	
	return atan(delta/cucumberHeight) * 180 / M_PI;
}

/**
 * Find the closest top to the specified x and y coordinate.
 * 
 * @param x: The x to look at
 * @param y: The y to look at
 * @param tops: A vector containing all the possible tops
 * @return The closest top from the input list, or one with width == -1 when no closest can be found.
 */
ros_faster_rcnn::Detection findClosestTop(float x, float y, std::vector<ros_faster_rcnn::Detection> tops) {
	ros_faster_rcnn::Detection closest = ros_faster_rcnn::Detection();
	closest.width = -1; 	// Flag value for no closest found
	float min_dist = std::numeric_limits<float>::max();
	
	for (ros_faster_rcnn::Detection d : tops) {
		float dist = std::sqrt(pow(d.x-x, 2) + pow(d.y-y, 2));
		if (dist < MAX_DIST && dist < min_dist) {
			min_dist = dist;
			closest = d;
		}
	}
	return closest;
}

/**
 * Processes the detection and produces a CucumberContainer object.
 */
CucumberContainer processDetection(ros_faster_rcnn::Detection det, std::vector<ros_faster_rcnn::Detection> tops) {	
	float width = det.width;
	float height = det.height;
	
	float x = det.x + (width * .5);
	float y = det.y;
	
	ros_faster_rcnn::Detection top = findClosestTop(x, y, tops);
	
	float curvature = -1;
	if (top.width > 0) {
		 curvature = calculateCurvature(top.width, det.width, det.height);
		 y = top.x + .5 * top.width;
	}
	
	CucumberContainer res = CucumberContainer(x, y, width, height, curvature);
	return res;
}

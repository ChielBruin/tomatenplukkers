#ifndef CucumberContainer_H
#define CucumberContainer_H

#include "cucumber_msgs/Cucumber.h"
#include "geometry_msgs/Pose.h"
#include <math.h>

/**
 * Container class that stores a cucumbers data.
 */
class CucumberContainer {
	float x, y, z;
	
	float width;
	float height;
	
	float curvature;

 public:
	/**
	 * Constructor from a cucumber_msgs::Cucumber message.
	 */
	CucumberContainer(cucumber_msgs::Cucumber msg) {
		this->x = msg.stem_position.x;
		this->y = msg.stem_position.y;
		this->z = msg.stem_position.z;
		this->width = msg.width;
		this->height = msg.height;
		this->curvature = msg.curvature;
	}

	/**
	 * Constructor for a 2D instance of the container object.
	 */
	CucumberContainer(float x, float y, float width, float height, float curvature) {
		this->x = x;
		this->y = y;
		this->z = 0;
		this->width = width;
		this->height = height;
		this->curvature = curvature;
	}
	
	/**
	 * Constructor for a 3D instance of the container object.
	 */
	CucumberContainer(float x, float y, float z, float width, float height, float curvature) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->width = width;
		this->height = height;
		this->curvature = curvature;
	}
	
	/**
	 * Converts the cucumber to a cucumber_msgs::Cucumber message.
	 * It leaves the header and dimension empty.
	 */
	cucumber_msgs::Cucumber toMessage() {
		cucumber_msgs::Cucumber msg;
		 msg.stem_position.x = x;
		 msg.stem_position.y = y;
		 msg.stem_position.z = z;
		 msg.width = width;
		 msg.height = height;
		 msg.curvature = curvature;
		return msg;		 
	}

	/**
	 * Generates a Pose for the location of the cucumber.
	 */
	geometry_msgs::Pose createPose() {
		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		pose.orientation.w = 1.0;
		return pose;
	}
	
	/**
	 * Get the weight of the cucumber in grams.
	 */
	float getWeight() {
		return M_PI * (this->width*.5) * (this->width*.5) * this->height;
	}
	
	/**
	 * Get the curvature of the cucumber.
	 */
	float getCurvature() {
		return this->curvature;
	}
};
#endif

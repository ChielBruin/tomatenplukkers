#ifndef CucumberContainer_H
#define CucumberContainer_H

#include "cucumber_msgs/Cucumber.h"

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
};
#endif

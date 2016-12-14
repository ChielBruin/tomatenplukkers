#ifndef CucumberContainer_H
#define CucumberContainer_H

#include "cucumber_msgs/Cucumber.h"
#include <math.h>

#define EQ_DIST .2
#define EQ_WEIGHT 30
#define EQ_CURVE 1

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
	
	/**
	 * Check if the specified other cucumber is equal to this cucumber.
	 * Equal, in this case, means that it lies within the bounds set by EQ_DIST, EQ_WEIGHT and EQ_CURVE.
	 * @param other [CucumberContainer]: the cucumber to check with
	 * @return True is they are equal, false otherwise
	 */
	bool equals(CucumberContainer other) {
		float dx = this->x - other.x;
		float dy = this->y - other.y;
		float dz = this->z - other.z;
		
		if (sqrt(dx*dx + dy*dy + dz*dz) > EQ_DIST) return false;
		if (fabs(this->getWeight() - other.getWeight()) > EQ_WEIGHT) return false;
		if (fabs(this->getCurvature() - other.getCurvature()) > EQ_CURVE) return false;
		
		return true;
	}
};
#endif

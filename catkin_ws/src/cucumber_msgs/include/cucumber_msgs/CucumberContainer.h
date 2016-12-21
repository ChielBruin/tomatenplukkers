#ifndef CucumberContainer_H
#define CucumberContainer_H

#include "cucumber_msgs/Cucumber.h"
#include "geometry_msgs/Pose.h"
#include <math.h>

#define DISTANCE_THRESHOLD .2
#define WEIGHT_THRESHOLD 30
#define CURVATURE_THRESHOLD 1

/**
 * Container class that stores a cucumbers data.
 */
class CucumberContainer {
	float x, y, z;
	
	float width;
	float height;
	
	float curvature;
	int image_x, image_y;

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
		this->image_x = msg.image_stem_position[0];
		this->image_y = msg.image_stem_position[1];
	}

	/**
	 * Constructor for a 2D instance of the container object.
	 */
	CucumberContainer(float x, float y, float width, float height, float curvature) {
		this->image_x = x;
		this->image_y = y;
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
		 std::vector<int> img_pos {image_x, image_y};
		 msg.image_stem_position = img_pos;
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
	
	/**
	 * Check if the specified other cucumber is equal to this cucumber.
	 * Equal, in this case, means that it lies within the bounds set by DISTANCE_THRESHOLD, WEIGHT_THRESHOLD and CURVATURE_THRESHOLD.
	 * The equality test does not use the original image positions as this is influenced by the used camera.
	 * @param other [CucumberContainer]: the cucumber to check with
	 * @return True is they are equal, false otherwise
	 */
	bool equals(CucumberContainer other) {
		float dx = this->x - other.x;
		float dy = this->y - other.y;
		float dz = this->z - other.z;
		
		if (sqrt(dx*dx + dy*dy + dz*dz) > DISTANCE_THRESHOLD) {
			return false;
		}
		if (fabs(this->getWeight() - other.getWeight()) > WEIGHT_THRESHOLD) {
			return false;
		}
		if (fabs(this->getCurvature() - other.getCurvature()) > CURVATURE_THRESHOLD) {
			return false;
		}
		
		return true;
	}
	
	/**
	 * Set the image position of this cucumber.
	 * @param x The x coordinate in the image
	 * @param y The y coordinate in the image
	 */
	void setImagePosition(int x, int y) {
		this->image_x = x;
		this->image_y = y;
	}
};
#endif

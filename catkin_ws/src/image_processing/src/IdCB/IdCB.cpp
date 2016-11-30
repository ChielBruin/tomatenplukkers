#include "cucumber_msgs/CucumberContainer.h"

#include "ros/ros.h"
#include "stereo_msgs/DisparityImage.h"
#include "cucumber_msgs/Cucumber.h"
#include "std_msgs/Time.h"

#define CAM_LEFT 0
#define CAM_RIGHT 1

stereo_msgs::DisparityImage disparity;

/**
 * Calculate the 3D properties of the cucumber from the given camera position and disparity image.
 */
CucumberContainer to3D(cucumber_msgs::Cucumber in, int camera, stereo_msgs::DisparityImage pointcloud) {
	CucumberContainer tmp = CucumberContainer(in.stem_position.x, in.stem_position.y, in.width, in.height, in.curvature);
	if(camera == CAM_LEFT) {
		return tmp;
	} else {
		return tmp;
	}
}

/**
 * Add a disparity image to the set.
 * This method also trims all the images on update.
 */
void setDisparityImage(stereo_msgs::DisparityImage newDisp) {
	disparity = newDisp;
}

/**
 * Get the disparity image that corresponds with the given timestamp.
 * Returns a null-pointer when the disparity cannot be found.
 */
stereo_msgs::DisparityImage getDisparityImage(ros::Time timestamp) {
	return disparity;
	// TODO time sync
}

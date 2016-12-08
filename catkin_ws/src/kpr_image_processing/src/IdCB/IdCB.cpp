#include "cucumber_msgs/CucumberContainer.h"
#include <map>

#include "ros/ros.h"
#include "stereo_msgs/DisparityImage.h"
#include "cucumber_msgs/Cucumber.h"
#include "std_msgs/Time.h"

#define CAM_LEFT 0
#define CAM_RIGHT 1

std::map<ros::Time, stereo_msgs::DisparityImage> disparity;

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
	std::pair<ros::Time, stereo_msgs::DisparityImage> p = std::pair<ros::Time, stereo_msgs::DisparityImage>(newDisp.header.stamp, newDisp);
	disparity.insert(p);
}

/**
 * Get the disparity image that corresponds with the given timestamp.
 * Returns a null-pointer when the disparity cannot be found.
 */
stereo_msgs::DisparityImage getDisparityImage(ros::Time timestamp) {
	//TODO: Check if the map is sorted old > new for speed reasons.
	if (disparity.empty()) {
		ROS_ERROR("The disparity image buffer is empty, that should not happen");
		return stereo_msgs::DisparityImage();
	}
	
	bool succes = false;
	stereo_msgs::DisparityImage res;
	
	for (auto it = disparity.cbegin(); it != disparity.cend();) {
		ros::Time time = it->first;
		if (time.sec == timestamp.sec && time.nsec == timestamp.nsec) {
			res = it->second;
			succes = true;
			break;
		}else if (time.sec < timestamp.sec) {
			it = disparity.erase(it);
		} else {
			++it;
		}
	}
	if (!succes) {
		ROS_WARN("No disparity image could be found for time=%d, using oldest instead.", timestamp.sec);
		return disparity.begin()->second;
	} else {
		return res;
	}
}

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
 * Z = distance along the camera Z axis (in metres)
 * f = focal length (in pixels)
 * B = baseline (in metres)
 * d = disparity (in pixels)
 * Z = fB/d
 * X = xZ/f (x is pixel-x-position)
 * Y = yZ/f (y is pixel-y-position)
 * Return: 3D postion cucumber(m) + width (m), height (m) and curvature
 */
CucumberContainer to3D(cucumber_msgs::Cucumber in, int camera, stereo_msgs::DisparityImage disparity) {
	CucumberContainer tmp = CucumberContainer(in);
	int x = in.image_stem_position[0];
	ROS_INFO("%f" , x);
	int y = in.image_stem_position[1];	
	float B = disparity.T;
	float pixel_size = 4.65e-6;
	float f = disparity.f;
	int d = disparity.image.data[disparity.image.step*y+48+x*disparity.image.step/disparity.image.width];
	if(camera == CAM_LEFT) {
		float Z = f*B/d;
		float X = x*Z/f;
		float Y = y*Z/f;
		float width = in.width*pixel_size;
		float height = in.height*pixel_size;
		float curvature = in.curvature;
		return CucumberContainer(X,Y,Z,width, height, curvature);
	} else {
		ROS_WARN("Right camera not supported");
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

#include "cucumber_msgs/CucumberContainer.h"
#include <map>

#include "ros/ros.h"
#include "stereo_msgs/DisparityImage.h"
#include "cucumber_msgs/Cucumber.h"
#include "std_msgs/Time.h"
#include <math.h>
#include <Eigen/Geometry>
#include <iostream>
#include <Eigen/Dense>

#define CAM_LEFT 0
#define CAM_RIGHT 1

std::map<ros::Time, stereo_msgs::DisparityImage> disparity;

/**
 * Calculate the 3D properties of the cucumber from the given camera position and disparity image.
 *
 * @param in Cucumber information obtaind from camera picture
 * @param camera The Left/Right camera used for the image using
 * @param disparity Information of the camera and the image made
 *
 * @return 3D postion cucumber(m), width (m), height (m) and curvature
 */
CucumberContainer to3D(cucumber_msgs::Cucumber in, int camera, stereo_msgs::DisparityImage disparity) {
	if(camera != CAM_LEFT) {
		ROS_WARN("Right camera not supported");
		return CucumberContainer(0, 0, 0, 0, -1);
	}

	int x = in.image_stem_position[0];
	int y = in.image_stem_position[1];
	float B = disparity.T;
	float pixel_size = 4.65e-6;
	float f = disparity.f;
	int im = disparity.image.step*(y+48)+x*disparity.image.step/disparity.image.width;

	if (im > disparity.image.data.size()){
		ROS_ERROR("Out of Range");
		return CucumberContainer(0, 0, 0, 0, -1);
	}

	int d = disparity.image.data[im];
	float Z_cam = f*B/(float)d;
	float X_cam = (x - 516)*Z_cam/f;
	float Y_cam = (y - 388)*Z_cam/f;
	float width = in.width*Z_cam/f;
	float height = in.height*Z_cam/f;
	float curvature = in.curvature;

	Eigen::Vector4f v(X_cam,Y_cam,Z_cam,1);
	
 	//translation and rotation degree
	Eigen::Vector3f translate(-0.16,0,0.1285);
	float rotDeg = 30;
	
	float rotRad = rotDeg*M_PI/180.0;
	Eigen::Transform<float,3,Eigen::Affine> transform = Eigen::Translation3f(translate) * Eigen::AngleAxisf(-rotRad,Eigen::Vector3f(1,0,0));
	v = transform*v;

	CucumberContainer res = CucumberContainer(v[0],v[1],v[2] + .5 * width ,width, height, curvature);
	res.setImagePosition(x,y);
	return res;
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
		} else if (time.sec < timestamp.sec) {
			res = it->second;
			succes = true;
			it = disparity.erase(it);
		} else {
			break;
		}
	}
	if (!succes) {
		ROS_WARN("No disparity image could be found for time=%d, using oldest instead.", timestamp.sec);
		return disparity.begin()->second;
	} else {
		return res;
	}
}

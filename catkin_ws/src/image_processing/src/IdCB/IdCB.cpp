#include "cucumber_msgs/CucumberContainer.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "cucumber_msgs/Cucumber.h"
#include "std_msgs/Time.h"

#define CAM_LEFT 0
#define CAM_RIGHT 1

sensor_msgs::PointCloud2 pointCloud;

/**
 * Calculate the 3D properties of the cucumber from the given camera position and pointcloud.
 */
CucumberContainer to3D(cucumber_msgs::Cucumber in, int camera, sensor_msgs::PointCloud2 pointcloud) {
	CucumberContainer tmp = CucumberContainer(in.stem_position.x, in.stem_position.y, in.width, in.height, in.curvature);
	if(camera = CAM_LEFT) {
		return tmp;
	} else {
		return tmp;
	}
}

/**
 * Add a point cloud to the set.
 * This method also trims all the old point clouds on update.
 */
void setPointCloud(sensor_msgs::PointCloud2 newPC) {
	pointCloud = newPC;
}

/**
 * Get the point cloud that corresponds with the given timestamp.
 * Returns a null-pointer when the point cloud cannot be found.
 */
sensor_msgs::PointCloud2 getPointCloud(ros::Time timestamp) {
	return pointCloud;
	// TODO time sync
}

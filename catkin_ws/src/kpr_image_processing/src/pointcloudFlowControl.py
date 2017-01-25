#!/usr/bin/python

import rospy
from sensor_msgs.msg import PointCloud2
from cucumber_msgs.msg import Cucumber

pointCloud = PointCloud2()

def pointCloudCallback(msg):
	'''
	Callback for receiving point clouds.
	Stores them at the global buffer spot.
	'''
	global pointCloud
	pointCloud = msg
	
def targetCallback(msg):
	'''
	Callback for receiving targets.
	When a target is received the last point cloud is published.
	'''
	global pointCloud, pointcloud_pub
	pointcloud_pub.publish(pointCloud)

if __name__ == '__main__':
	'''
	Main function that starts the ROS subscribers and publishers.
	'''
	rospy.init_node('pointcloudFlowControl')
	pointcloud_sub = rospy.Subscriber("/camera/points2", PointCloud2, pointCloudCallback)
	target_sub = rospy.Subscriber("/target", Cucumber, targetCallback)
	pointcloud_pub = rospy.Publisher("/points2", PointCloud2, queue_size=10)
	
	rospy.loginfo("Started")
	rospy.spin()
	rospy.loginfo("Stopped")

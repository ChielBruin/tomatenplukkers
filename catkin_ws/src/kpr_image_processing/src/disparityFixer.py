#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()
kernel = np.ones((5,5), np.uint8)

def disparityCallback(msg):
	'''
	Callback function for the disparity images.
	Applies an OpenCV closing operation on the received image and publishes the result.
	'''
	global disparity_pub, bridge, kernel
	try:
		img = bridge.imgmsg_to_cv2(msg.image, "bgr8")
		closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
		msg.image = bridge.cv2_to_imgmsg(closing, "bgr8")
	except CvBridgeError as e:
		rospy.logerr("Error converting images: %s", e)

	disparity_pub.publish(msg)

if __name__ == '__main__':
	'''
	Main function that starts the ROS subscribers and publishers.
	'''
	rospy.init_node('disparityFixer')
	image_sub = rospy.Subscriber("/camera/disparity", DisparityImage, disparityCallback)
	disparity_pub = rospy.Publisher("/disparity", DisparityImage)
	
	rospy.loginfo("Started")
	rospy.spin()
	rospy.loginfo("Stopped")

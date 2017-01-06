#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()
kernel = np.ones((15,15), np.uint8)

#cv2.startWindowThread()
#cv2.namedWindow('before', cv2.WINDOW_NORMAL)
#cv2.startWindowThread()
#cv2.namedWindow('after', cv2.WINDOW_NORMAL)

def disparityCallback(msg):
	'''
	Callback function for the disparity images.
	Applies an OpenCV closing operation on the received image and publishes the result.
	'''
	global disparity_pub, bridge, kernel
	try:
		img = bridge.imgmsg_to_cv2(msg.image, desired_encoding="8UC1")
		closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
		#cv2.imshow('before',img)
		#cv2.imshow('after',closing)
		msg.image = bridge.cv2_to_imgmsg(closing)
	except CvBridgeError as e:
		rospy.logerr("Error converting images: %s", e)
		
	disparity_pub.publish(msg)

if __name__ == '__main__':
	'''
	Main function that starts the ROS subscribers and publishers.
	'''
	rospy.init_node('disparityFixer')
	image_sub = rospy.Subscriber("/camera/disparity", DisparityImage, disparityCallback)
	disparity_pub = rospy.Publisher("/disparity", DisparityImage, queue_size=10)
	
	rospy.loginfo("Started")
	rospy.spin()
	rospy.loginfo("Stopped")

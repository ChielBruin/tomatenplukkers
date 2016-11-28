#!/usr/bin/python

import rospy
from ur_msgs.srv import *

def request(req):
	print "%i %i %f" %(req.fun, req.pin, req.state)
	response = raw_input("response (True/False) =")
	return SetIOResponse(response)


rospy.init_node('io_server')
s = rospy.Service('io', SetIO, request)
rospy.spin()



#!/usr/bin/python

import rospy
from ur_msgs.srv import *

def request(req):
	print "%i %i %f" %(req.fun, req.pin, req.state)
	res = response()	
	return SetIOResponse(res)

def response():
	try:
		res = int(input("response (1/0) ="))
		if res == 1 or res == 0:
			return bool(res)
		else:
			print "Enter a 1 or 0"
			return response()
	except NameError:		
		print "Not a number"
		return response()


rospy.init_node('io_server')
s = rospy.Service('io', SetIO, request)
rospy.spin()



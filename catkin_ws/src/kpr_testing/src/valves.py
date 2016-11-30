#!/usr/bin/python

import rospy, os
from distutils.util import strtobool
from ur_msgs.srv import *

header = """
Check the status of I/O-'poortjes'
"""

def display():
	os.system('clear')
	print header

#These IO pins are used
pinA = 1
pinB = 2

#Checking if used pins are correct
#DIT kan NIET, wegens req = not defined
def pin_check(pin):
	if (pin == pinA) or (pin == pinB):
		print "correct io pins used"
		return True
	else:
		print "incorrect io pins used"
		return False

def request(req):
	pin = req.pin
	print "Type req.pin ",type(req.pin)," type pin ", type(pin)
	print "fun is %i" %(req.fun)
	print "pin is %i" %(req.pin)
	print "state is %f \n" %(req.state)
	#print "pin A is ", pinA
	#print "pin B is ", pinB
	print pin_check(pin)
	res = response()	
	return SetIOResponse(res)

#What you want to give as succes response
#At the moment it gives a Syntax Error if you type 1d for example
def response():
	try:
		return strtobool(raw_input("response (True/False) = ").lower())
	except ValueError:		
		print "\nNot a number"
		return response()
		



rospy.init_node('io_server')
s = rospy.Service('io', SetIO, request)
display()
#pin_check()
rospy.spin()



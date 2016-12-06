#!/usr/bin/python

import rospy, os, tty, termios, select, sys
from distutils.util import strtobool
from ur_msgs.srv import *
from ur_msgs.msg import *

header = """
Check the status of I/O-pins
"""

def display():
	os.system('clear')
	print header
#Making table
	a = "Pin     |  State     Fun     "
	print a
	print "-"*len(a)
	print "VacSens | ", info[VacSens][0], "  ", info[VacSens][1]
	print "Gripper | ", info[Gripper][0], "  ", info[Gripper][1]

#These IO pins are used
VacSens = "DI1"
Gripper = "DO2"

#dictionary of pins
info = {VacSens:["-","     -"], Gripper:["-","     -"]}

#Checking if used pins are correct
def pin_check(pin):
	if (pin == VacSens) or (pin == Gripper):
		print "correct io pins used"
		return True
	else:
		print "incorrect io pins used"
		return False



def request(req):
#	display()
	if req.fun == 1:
		pin = 'DO' + str(req.pin)
	elif req.fun == 4:
		pin = 'AO' + str(req.pin)
	else:
		rospy.logerr("Writing to unused pin")
		return SetIOResponse(False)
#Change Dictionary "info" with correct data
	if pin not in info:
		rospy.logerr("Writing to unused pin")
		return SetIOResponse(False)
	info[pin][0] = req.state
	if req.fun == 1:
		info[pin][1] = "Digital Out"
	else:
		info[pin][1] = "Analog Out"
#	print info
#	print "fun is %i" %(req.fun)
#	print "pin is %i" %(req.pin)
#	print "state is %f \n" %(req.state)
#prints if the correct pins are used
#	print pin_check(pin)
	display()
	res = response()	
	return SetIOResponse(res)

#What you want to give as succes response
def response():
	try:
		return strtobool(raw_input("response (True/False) = ").lower())
	except ValueError:		
		print "\nNot a boolean\n"
		return response()
		
def get_message():
	msg = IOStates()
	for pin in info:
		if info[pin][1] == "Digital Out":
			msg.digital_out_states.append(Digital(pin, bool(info[pin][0])))
		elif info[pin][1] == "Analog Out":
			msg.analog_out_states.append(Analog(pin, info[pin][0]))
	return msg

#def getKey():
#	tty.setraw(sys.stdin.fileno())
#	select.select([sys.stdin], [], [], 0)
#	key = sys.stdin.read(1)
#	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#	return key


#settings = termios.tcgetattr(sys.stdin)
rospy.init_node('io_server')
s = rospy.Service('io', SetIO, request)
display()
io_state_pub = rospy.Publisher("io_states", IOStates, queue_size=10)
rate = rospy.Rate(1) #1Hz

while not rospy.is_shutdown():
	io_state_pub.publish(get_message())
#	getKey()
	rate.sleep()

#termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

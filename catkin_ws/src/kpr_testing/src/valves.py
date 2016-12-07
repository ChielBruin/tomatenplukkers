#!/usr/bin/python

import rospy, termios, sys, contextlib, os
import numpy as np
from ur_msgs.srv import *
from ur_msgs.msg import *

header = """
Check the status of I/O-pins
\t-^D to exit
\t-i to change income settings
\t-p to send io states
\t-t to toggle service succes
"""

def display():
	os.system('clear')
	print header
#Making table
	tableHeader = "Pin     |  State     Fun     "
	print tableHeader
	print "-"*len(tableHeader)
	print "VacSens | ", info[VacSens][0], "  ", info[VacSens][1]
	print "Gripper | ", info[Gripper][0], "  ", info[Gripper][1]

#These IO pins are used
VacSens = "DI1"
Gripper = "DO2"

#Set succes response
response = True

#dictionary of pins
info = {VacSens:["-","     -"], Gripper:["-","     -"]}

#needed for the "pressing keys"
@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def request(req):
	global response
	display()
#Checking if changes wil be made to used pins 
	if req.fun == 1:
		pin = 'DO' + str(req.pin)
	elif req.fun == 4:
		pin = 'AO' + str(req.pin)
	else:
		rospy.logerr("Writing to unused pin")
		return SetIOResponse(False)
	if pin not in info:
		rospy.logerr("Writing to unused pin")
		return SetIOResponse(False)
#Change Dictionary "info" with correct data
	info[pin][0] = req.state
	if req.fun == 1:
		info[pin][1] = "Digital Out"
	else:
		info[pin][1] = "Analog Out"

	display()
	res = response	
	return SetIOResponse(res)
		
def get_message():
	msg = IOStates()
	for pin in info:
		if info[pin][1] == "Digital Out":
			msg.digital_out_states.append(Digital(int(pin[2]), bool(info[pin][0])))
		elif info[pin][1] == "Analog Out":
			msg.analog_out_states.append(Analog(int(pin[2]), float(info[pin][0])))
		elif info[pin][1] == "Digital In":
			msg.digital_in_states.append(Digital(int(pin[2]), bool(info[pin][0])))
		elif info[pin][1] == "Analog In":
			msg.analog_in_states.append(Analog(int(pin[2]), float(info[pin][0])))
	return msg

def processKey(key, pub):
	global response
	if key == chr(116): #t
		response = not response
		display()
		print "Your response is now: ", response
		return
	if key == chr(112): #p
		pub.publish(get_message())
		display()
		print "you have published the data"
		return
	if key == chr(105): #i
		display()
		print "Changing the input settings"
		income()
		return
	rospy.logwarn('unknown function %s', key)

def income():
	try:
		fun = int(raw_input("What is the fun? "))
		if fun != 5 and fun != 6:
			display()
			rospy.logerr("Fun nr not available")
			return SetIOResponse(False)
		print fun
	except ValueError:
		rospy.logerr("Not a Number")
		return SetIOResponse(False)

	pinQ = str(raw_input("Which pin you want to set? " ))
	print pinQ

	if fun == 5:
		pin = 'DI' + pinQ
	elif fun == 6:
		pin = 'AI' + pinQ
	else:
		rospy.logerr("Writing to unused pin")
		return SetIOResponse(False)
#Checks if pin in dictionary
	if pin not in info:
		rospy.logerr("Writing to unused pin")
		return SetIOResponse(False)
	print "writing to pin ", pin
	state = float(raw_input("What is the state? "))
#Send state to dictionary 
	info[pin][0] = state 
#Send fun to dictionary
	if fun == 5: #fun
		info[pin][1] = "Digital In"
	else:
		info[pin][1] = "Analog In"
	display()
	return 

def main():
	rospy.init_node('io_server')
	rospy.loginfo("Started")
	s = rospy.Service('io', SetIO, request)
	display()
	io_state_pub = rospy.Publisher("io_states", IOStates, queue_size=10)
	rate = rospy.Rate(1) #1Hz

	with raw_mode(sys.stdin):
		try:
			while not rospy.is_shutdown():
				ch = sys.stdin.read(1)
				if not ch or ch == chr(4):
					break
				processKey(ch, io_state_pub)
		except (KeyboardInterrupt, EOFError):
			pass
	rospy.loginfo("Stopped")

if __name__ == '__main__':
	main()

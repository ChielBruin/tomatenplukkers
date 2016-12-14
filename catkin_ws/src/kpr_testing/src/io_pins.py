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

#Set succes response
response = True

#dictionary of pins
pinStates = {	
	"DI1":["-   ","-","VacSens"], 
	"DO2":["-   ","-","Gripper"],
	"DO4":["-	","-","Cutter"],
}

def display():
	'''
	Makes a table with all the IO pinStatesrmation
	gathered from the pins dictionary: pinStates
	'''
	os.system('clear')
	print header
#Making table
	tableHeader = "Pin \t \t | State \t Fun     "
	print tableHeader
	print "-"*len(tableHeader.expandtabs(8))
	for entry in pinStates:
		e = pinStates[entry]
		print("{}({}) \t | {} \t {}".format(entry, e[2], e[0], e[1]))

@contextlib.contextmanager
def raw_mode(file):
	'''
	fancy shizzle to wait till a keyboard key has been pressed
	which it returns
	''' 
	old_attrs = termios.tcgetattr(file.fileno())
	new_attrs = old_attrs[:]
	new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
	try:
		termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
		yield
	finally:
		termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

def request(req):
	'''
	Set io data to dictionary 'pinStates'
	req: comes from the ur_msgs.srv
		these inputs can be changed by calling:
		rosservice call /io "fun: #value
		pin: # value
		state: #value "
	fun has a value of 1 (Digital) or 4 (Analog)
	pins has a number between 1 and 8
		not all pins are used
	state is between 0.0 and 1.0
	
	Returns: succes response (False/True)

	Quit help function: q
	'''
	global response
#Checking if changes wil be made to used pins 
	if req.fun == 1:
		pin = 'DO' + str(req.pin)
	elif req.fun == 4:
		pin = 'AO' + str(req.pin)
	else:
		rospy.logerr("%d is no valid fun value, only 1 (Digital Out) and 4 (Analog Out) allowed", req.fun)
		return SetIOResponse(False)
	if pin not in pinStates:
		rospy.logerr("Writing to unused pin")
		return SetIOResponse(False)
#Change dictionary "pinStates" with correct data
	if req.fun == 1:
		pinStates[pin][1] = "Digital Out"
		if (req.state != 0 and req.state != 1):
			rospy.logerr("State value is %f, but must be either 0 or 1.", req.state)
			return SetIOResponse(False)
	else:
		pinStates[pin][1] = "Analog Out"
		if (req.state < 0 or req.state > 1):
			rospy.logerr("State value is %f, but must be between 0 and 1.", req.state)
			return SetIOResponse(False)
	pinStates[pin][0] = "{0:.2f}".format(req.state)

	display()
	res = response	
	return SetIOResponse(res)
		
def get_message():
	'''
	publish the io data from the dictionary
	to 4 different lists:
	Digital In
	Digital Out
	Analog In
	Analog Out
	'''
	msg = IOStates()
	for pin in pinStates:
		if pinStates[pin][1] == "Digital Out":
			msg.digital_out_states.append(Digital(int(pin[2]), bool(pinStates[pin][0])))
		elif pinStates[pin][1] == "Analog Out":
			msg.analog_out_states.append(Analog(int(pin[2]), float(pinStates[pin][0])))
		elif pinStates[pin][1] == "Digital In":
			msg.digital_in_states.append(Digital(int(pin[2]), bool(pinStates[pin][0])))
		elif pinStates[pin][1] == "Analog In":
			msg.analog_in_states.append(Analog(int(pin[2]), float(pinStates[pin][0])))
	return msg

def processKey(key, pub):
	'''
	Checks if an important key has been pressed
	t: changes succes response True <-> False
	p: publish the data, by using the get_message() function
	i: change the input settings, uses updateInputPinValues()
	'''
	global response
	if key == chr(116): #t
		response = not response
		display()
		print "Your response is now: ", response
		return
	if key == chr(112): #p
		pub.publish(get_message())
		display()
		print "You have published the data"
		return
	if key == chr(105): #i
		display()
		print "Changing the input settings"
		updateInputPinValues()
		return
	rospy.logwarn('Unknown function %s', key)

def updateInputPinValues():
	'''
	Change the input settings and stores it in the
		dictionary 'pinStates'
	fun: a value of 5 (Digital) or 6 (Analog)
	pin: number between 1 and 8
	state: value between 0.0 and 1.0
	
	Quit help function by pressing 'Q'
	'''
	try:
		fun = int(raw_input("What is the fun? "))
		if fun != 5 and fun != 6:
			display()
			rospy.logerr("Fun nr not available")
			help(updateInputPinValues)
			return
		print fun
	except ValueError:
		rospy.logerr("Not a Number")
		return

	pinQ = str(raw_input("Which pin you want to set? " ))
	print pinQ

	if fun == 5:
		pin = 'DI' + pinQ
	elif fun == 6:
		pin = 'AI' + pinQ
#Checks if pin in dictionary
	if pin not in pinStates:
		rospy.logerr("Writing to unused pin")
		return
	print "writing to pin ", pin
	state = float(raw_input("What is the state? "))
#Send fun to dictionary
	if fun == 5: #fun
		if (state != 0 and state != 1):
			rospy.logerr("State value is %f, but must be either 0 or 1.", state)
			return
		pinStates[pin][1] = "Digital In"
	else:
		if (state < 0 or state > 1):
			rospy.logerr("State value is %f, but must be between 0 and 1.", state)
			return
		pinStates[pin][1] = "Analog In"
		
#Send state to dictionary 
	pinStates[pin][0] = "{0:.2f}".format(state)
	
	display()
	return 

def main():
	'''
	main function
	needed for running this service
	'''
	rospy.init_node('io_server')
	rospy.loginfo("Started")
	s = rospy.Service('set_io_testing', SetIO, request)
	display()
	io_state_pub = rospy.Publisher("io_states", IOStates, queue_size=10)


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

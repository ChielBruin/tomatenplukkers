#!/usr/bin/python

import rospy, termios, sys, contextlib, os
from distutils.util import strtobool
from ur_msgs.srv import *
from ur_msgs.msg import *

header = """
Check the status of I/O-pins
\t-^D to exit
\t-t to toggle service succes
\t-p to send io states
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

response = True

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
	print info
	print "fun is %i" %(req.fun)
	print "pin is %i" %(req.pin)
	print "state is %f \n" %(req.state)
#prints if the correct pins are used
	print pin_check(pin)
	display()
	res = response	
	return SetIOResponse(res)
		
def get_message():
	msg = IOStates()
	for pin in info:
		if info[pin][1] == "Digital Out":
			msg.digital_out_states.append(Digital(pin, bool(info[pin][0])))
		elif info[pin][1] == "Analog Out":
			msg.analog_out_states.append(Analog(pin, info[pin][0]))
	return msg

def processKey(key, pub):
	global response
	if key == chr(116):
		response = not response
		return
	if 	key == chr(112):
		pub.publish(get_message())
		return
	rospy.logwarn('unknown function %s', key)

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

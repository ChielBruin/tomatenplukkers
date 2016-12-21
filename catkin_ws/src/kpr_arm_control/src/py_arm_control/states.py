#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Quaternion, Pose

# Output pins
VACUUM_OUT = 1
CUTTER_OUT = 2
GRIPPER_OUT = 3

# Input pins
VACUUM_IN = 1
CUTTER_IN = 2
GRIPPER_IN = 3

GRIPPER_CLOSE = False
GRIPPER_OPEN = True
CUTTER_CLOSE = True
CUTTER_OPEN = False
VACUUM_ON = True
VACUUM_OFF = False

class MoveToCucumber(smach.State):
	def __init__(self, moveArmTo):
		self.moveArmTo = moveArmTo
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], input_keys=['data'])

	def execute(self, userdata):
		rospy.loginfo('Executing state MoveToCucumber')
		pose = Pose(userdata.data.cucumber.stem_position, Quaternion(0,0,0,1))
		if self.moveArmTo(pose):
			return 'MoveOK'
		else:
			return 'MoveError'

class CloseGripper(smach.State):
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['GripperClosed', 'GripperError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state CloseGripper')
		if self.setIO(GRIPPER_OUT, GRIPPER_CLOSE, GRIPPER_IN, GRIPPER_CLOSE):
			return 'GripperClosed'
		else:
			return 'GripperError'

class VacuumGrip(smach.State):
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['VacuumCreated', 'VacuumError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state VacuumGrip')
		if self.setIO(VACUUM_OUT, VACUUM_ON, VACUUM_IN, VACUUM_ON):
			return 'VacuumCreated'
		else:
			return 'VacuumError'

class Cut(smach.State):
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['StemCutted', 'CutterError'])
		
	def execute(self, userdata):
		rospy.loginfo('Executing state Cut')
		if self.setIO(CUTTER_OUT, CUTTER_CLOSE, CUTTER_IN, CUTTER_CLOSE):
			if self.setIO(CUTTER_OUT, CUTTER_OPEN, CUTTER_IN, CUTTER_OPEN):
				return 'StemCutted'
		return 'CutterError'

class MoveToDropoff(smach.State):
	def __init__(self, moveArmTo):
		self.moveArmTo = moveArmTo
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], input_keys=['data'])

	def execute(self, userdata):
		rospy.loginfo('Executing state MoveToDropoff')
		if self.moveArmTo(userdata.data.dropLocation):
			return 'MoveOK'
		else:
			return 'MoveError'

class OpenGripper(smach.State):
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['GripperOpened', 'GripperError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state OpenGripper')
		if self.setIO(GRIPPER_OUT, GRIPPER_OPEN, GRIPPER_IN, GRIPPER_OPEN):
			return 'GripperOpened'
		else:
			return 'GripperError'

class MoveToStart(smach.State):
	def __init__(self, moveArmTo):
		self.moveArmTo = moveArmTo
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], input_keys=['start'])

	def execute(self, userdata):
		rospy.loginfo('Executing state MoveToStart')
		if self.moveArmTo(userdata.start):
			return 'MoveOK'
		else:
			return 'MoveError'

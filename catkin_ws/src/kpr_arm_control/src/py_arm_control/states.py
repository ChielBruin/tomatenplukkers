#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Quaternion, Pose

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
	def __init__(self):
		smach.State.__init__(self, outcomes=['GripperClosed', 'GripperError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state CloseGripper')
		if True:
			return 'GripperClosed'
		else:
			return 'GripperError'

class VacuumGrip(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['VacuumCreated', 'VacuumError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state VacuumGrip')
		if True:
			return 'VacuumCreated'
		else:
			return 'VacuumError'

class Cut(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['StemCutted', 'CutterError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Cut')
		if True:
			return 'StemCutted'
		else:
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
	def __init__(self):
		smach.State.__init__(self, outcomes=['GripperOpened', 'GripperError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state OpenGripper')
		if True:
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

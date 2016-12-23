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
	'''
	State representing the movement towards the cucumber.
	'''
	def __init__(self, moveArmTo):
		self.moveArmTo = moveArmTo
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], input_keys=['data'])

	def execute(self, userdata):
		'''
		Move the arm to the position of the cucumber and exit this state when reached
		
		@return 'MoveOK' when the move was successful, 'MoveError' otherwise
		'''
		rospy.loginfo('Executing state MoveToCucumber')
		pose = Pose(userdata.data.cucumber.stem_position, Quaternion(0,0,0,1))
		# TODO: Make the last section of movement straight towards the produce
		if self.moveArmTo(pose):
			return 'MoveOK'
		else:
			return 'MoveError'

class CloseGripper(smach.State):
	'''
	State representing the the closing of the gripper.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['GripperClosed', 'GripperError'])

	def execute(self, userdata):
		'''
		Close the gripper and check if this is actualy done.
		
		@return 'GripperClosed' when successful, 'GripperError' otherwise
		'''
		rospy.loginfo('Executing state CloseGripper')
		if self.setIO(GRIPPER_OUT, GRIPPER_CLOSE, GRIPPER_IN, GRIPPER_CLOSE):
			return 'GripperClosed'
		else:
			return 'GripperError'

class VacuumGrip(smach.State):
	'''
	State representing the gripping of the cucumber using the vacuum suction cup.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['VacuumCreated', 'VacuumError'])

	def execute(self, userdata):
		'''
		Enable the vacuum and make sure it is correctly initiated.
		
		@return 'VacuumCreated' when the vacuum is created succesfully, 'VacuumError' otherwise
		'''
		rospy.loginfo('Executing state VacuumGrip')
		if self.setIO(VACUUM_OUT, VACUUM_ON, VACUUM_IN, VACUUM_ON):
			return 'VacuumCreated'
		else:
			# TODO: Tilt wrist and try again using a new state
			return 'VacuumError'

class Cut(smach.State):
	'''
	State representing the cutting of the stem of the cucumber.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['StemCutted', 'CutterError'])
		
	def execute(self, userdata):
		'''
		Cut the stem by closing and opening the cutter.
		
		@return 'StemCutted' when the cutting was successful, 'CutterError' otherwise
		'''
		rospy.loginfo('Executing state Cut')
		if self.setIO(CUTTER_OUT, CUTTER_CLOSE, CUTTER_IN, CUTTER_CLOSE):
			if self.setIO(CUTTER_OUT, CUTTER_OPEN, CUTTER_IN, CUTTER_OPEN):
				return 'StemCutted'
		return 'CutterError'

class MoveToDropoff(smach.State):
	'''
	State representing the arm movement towards the dropoff location
	'''
	def __init__(self, moveArmTo):
		self.moveArmTo = moveArmTo
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], input_keys=['data'])

	def execute(self, userdata):
		'''
		Move the arm to the dropoff location provided by the input data.
		
		@return 'MoveOK' when the move was successful, 'MoveError' otherwise
		'''
		rospy.loginfo('Executing state MoveToDropoff')
		if self.moveArmTo(userdata.data.dropLocation):
			return 'MoveOK'
		else:
			return 'MoveError'

class OpenGripper(smach.State):
	'''
	State representing the opening of the gripper and releasing the produce.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['GripperOpened', 'GripperError'])

	def execute(self, userdata):
		'''
		Drop the produce by opeing the gripper and removing the vacuum.
		
		@return 'GripperOpened' when the produce is dropped correctly, 'GripperError' otherwise
		'''
		rospy.loginfo('Executing state OpenGripper')
		# TODO: Swap these according to the dropping procedure, 
		# this order seems fine for horizontal dropping in a crate.
		if self.setIO(GRIPPER_OUT, GRIPPER_OPEN, GRIPPER_IN, GRIPPER_OPEN):
			if self.setIO(VACUUM_OUT, VACUUM_OFF, VACUUM_IN, VACUUM_OFF):
				return 'GripperOpened'
		return 'GripperError'

class MoveToStart(smach.State):
	'''
	State representing the movement of the arm while returning to its initial position.
	'''
	def __init__(self, moveArmTo):
		self.moveArmTo = moveArmTo
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], input_keys=['start'])

	def execute(self, userdata):
		'''
		Move the arm to the initial starting location provided by the input data.
		
		@return 'MoveOK' when the move was successful, 'MoveError' otherwise
		'''
		# TODO: This state should be executed when something goes wrong, but this needs to be discussed
		rospy.loginfo('Executing state MoveToStart')
		if self.moveArmTo(userdata.start):
			return 'MoveOK'
		else:
			return 'MoveError'

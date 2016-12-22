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
		smach.State.__init__(self, outcomes=['GripperClosed', 'GripperFail', 'GripperError'], output_keys=['gripperStatus'])
		self.tries = 0
		self.maxTries = 3

	def execute(self, userdata):
	'''
	Close the gripper and check if this is actualy done.
	
	@return 'GripperClosed' when successful, 'GripperError' otherwise
	'''
		rospy.loginfo('Executing state CloseGripper')
		if self.setIO(GRIPPER_OUT, GRIPPER_CLOSE, GRIPPER_IN, GRIPPER_CLOSE):
			return 'GripperClosed'
		else:
			if self.tries < self.maxTries:
				self.tries++
				return 'GripperFail'
			userdata.gripperStatus = 'GRAB_ERR'
			return 'GripperError'

class RepositionGripper(smach.State):
	def __init__(self, moveArmTo, group):
		self.moveArmTo = moveArmTo
		self.group = group
		smach.State.__init__(self, outcomes=['Repositioned', 'RepositionFailed'])

	def execute(self, userdata):
		rospy.loginfo('Executing state RepositionGripper')
		pose = group.get_current_pose().pose	# TODO: Calculate the new position
		if moveArmTo(pose):
			return 'Repositioned'
		else:
			return 'RepositionFailed'

class VacuumGrip(smach.State):
'''
State representing the gripping of the cucumber using the vacuum suction cup.
'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['VacuumCreated', 'VacuumFail', 'VacuumError'], output_keys=['vacuumStatus'])

	def execute(self, userdata):
	'''
	Enable the vacuum and make sure it is correctly initialted.
	
	@return 'VacuumCreated' when the vacuum is created succesfully, 'VacuumError' otherwise
	'''
		rospy.loginfo('Executing state VacuumGrip')
		if self.setIO(VACUUM_OUT, VACUUM_ON, VACUUM_IN, VACUUM_ON):
			return 'VacuumCreated'
		else:
			if self.tries < self.maxTries:
				self.tries++
				return 'VacuumFail'
			userdata.vacuumStatus = 'VACC_ERR'
			return 'VacuumError'

class Tilt(smach.State):
	def __init__(self, moveArmTo, group):
		self.moveArmTo = moveArmTo
		self.group = group
		smach.State.__init__(self, outcomes=['TiltOK', 'TiltError'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Tilt')
		pose = group.get_current_pose().pose	# TODO: Calculate the new position
		if moveArmTo(pose):
			return 'TiltOK'
		else:
			return 'TiltError'
			
class Cut(smach.State):
'''
State representing the cutting of the stem of the cucumber.
'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['StemCutted', 'CutterOpenError', 'CutterCloseError'], output_keys=['cutterStatus'])
		
	def execute(self, userdata):
	'''
	Cut the stem by closing and opening the cutter.
	
	@return 'StemCutted' when the cutting was successful, 'CutterError' otherwise
	'''
		rospy.loginfo('Executing state Cut')
		if self.setIO(CUTTER_OUT, CUTTER_CLOSE, CUTTER_IN, CUTTER_CLOSE):
			if self.setIO(CUTTER_OUT, CUTTER_OPEN, CUTTER_IN, CUTTER_OPEN):
				return 'StemCutted'
			else:
				return 'CutterOpenError'
			userdata.cutterStatus = 'CUTT_ERR'
		return 'CutterCloseError'

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

class Release(smach.State):
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['ReleasedAll', 'GripperError', 'VacuumError', 'CutterError', 'ReleaseError'], input_keys=['systemStatus'])
		self.result = {
					'OK': 'ReleasedAll',
					'GRAB_ERR': 'GripperError', 
					'VACC_ERR': 'VacuumError', 
					'CUTT_ERR': 'CutterError'}

	def execute(self, userdata):
		rospy.loginfo('Executing state Release')
		# TODO: Swap these according to the dropping procedure, 
		# this order seems fine for horizontal dropping in a crate.
		if self.setIO(GRIPPER_OUT, GRIPPER_OPEN, GRIPPER_IN, GRIPPER_OPEN):
			if self.setIO(VACUUM_OUT, VACUUM_OFF, VACUUM_IN, VACUUM_OFF):
				return self.result[userdata.systemStatus]
		return 'ReleaseError'

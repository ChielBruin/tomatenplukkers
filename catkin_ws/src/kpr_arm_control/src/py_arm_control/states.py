#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Quaternion, Pose
from cucumber_msgs.msg import HarvestStatus

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
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], input_keys=['data', 'result'], output_keys=['result'])

	def execute(self, userdata):
		'''
		Move the arm to the position of the cucumber and exit this state when reached
		
		@return 'MoveOK' when the move was successful, 'MoveError' otherwise
		'''
		rospy.loginfo('Executing state MoveToCucumber')
		pose = Pose(userdata.data.cucumber.stem_position, Quaternion(0,0,0,1))
		# TODO: Make the last section of movement straight towards the produce
		(plan, move) = self.moveArmTo(pose)
		if not plan:
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error planning the movement')
			return 'MoveError'
		if move:
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			return 'MoveOK'
		else:
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error moving to the target')
			return 'MoveError'

class CloseGripper(smach.State):
	'''
	State representing the the closing of the gripper.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['GripperClosed', 'GripperFail', 'GripperError'], 
			input_keys=['result'], output_keys=['gripperStatus', 'result'])
		self.tries = 0
		self.maxTries = 3

	def execute(self, userdata):
		'''
		Close the gripper and check if this is actualy done.
		
		@return 'GripperClosed' when successful, 'GripperError' otherwise
		'''
		rospy.loginfo('Executing state CloseGripper')
		if self.setIO(GRIPPER_OUT, GRIPPER_CLOSE, GRIPPER_IN, GRIPPER_CLOSE):
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			return 'GripperClosed'
		else:
			if self.tries < self.maxTries:
				self.tries += 1
				return 'GripperFail'
			self.tries = 0
			userdata.result.grip = HarvestStatus(success = HarvestStatus.ERROR, message = 'Cannot grab the cucumber')
			userdata.gripperStatus = 'GRAB_ERR'
			return 'GripperError'

class RepositionGripper(smach.State):
	'''
	State representing the repositioning of the gripper when the gripping action fails.
	'''
	def __init__(self, moveArmTo, group):
		self.moveArmTo = moveArmTo
		self.group = group
		smach.State.__init__(self, outcomes=['Repositioned', 'RepositionFailed'])

	def execute(self, userdata):
		'''
		Mov the gripper away from the produce.
		
		@return 'Repositioned' when successful, 'RepositionFailed' otherwise
		'''
		rospy.loginfo('Executing state RepositionGripper')
		pose = group.get_current_pose().pose	# TODO: Calculate the new position
		if moveArmTo(pose)[1]:
			return 'Repositioned'
		else:
			return 'RepositionFailed'

class VacuumGrip(smach.State):
	'''
	State representing the gripping of the cucumber using the vacuum suction cup.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['VacuumCreated', 'VacuumFail', 'VacuumError'], 
			input_keys=['result'], output_keys=['vacuumStatus', 'result'])
		self.tries = 0
		self.maxTries = 3

	def execute(self, userdata):
		'''
		Enable the vacuum and make sure it is correctly initiated.
		
		@return 'VacuumCreated' when the vacuum is created succesfully, 'VacuumError' otherwise
		'''
		rospy.loginfo('Executing state VacuumGrip')
		if self.setIO(VACUUM_OUT, VACUUM_ON, VACUUM_IN, VACUUM_ON):
			userdata.result.grip = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			return 'VacuumCreated'
		else:
			if self.tries < self.maxTries:
				self.tries += 1
				return 'VacuumFail'
			self.tries = 0
			userdata.result.grip = HarvestStatus(success = HarvestStatus.ERROR, message = 'Cannot create vacuum')
			userdata.vacuumStatus = 'VACU_ERR'
			return 'VacuumError'

class Tilt(smach.State):
	'''
	State representing the wrist tilt that is used when the suction cup does not 'grab' the cucumber.
	'''
	def __init__(self, moveArmTo, group):
		self.moveArmTo = moveArmTo
		self.group = group
		smach.State.__init__(self, outcomes=['TiltOK', 'TiltError'])

	def execute(self, userdata):
		'''
		Tilt the wrist of the arm.
		
		@return 'TiltOK' when successful, 'TiltError' otherwise
		'''
		rospy.loginfo('Executing state Tilt')
		pose = self.group.get_current_pose().pose	# TODO: Calculate the new position
		if self.moveArmTo(pose)[1]:
			return 'TiltOK'
		else:
			return 'TiltError'
			
class Cut(smach.State):
	'''
	State representing the cutting of the stem of the cucumber.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['StemCutted', 'CutterOpenError', 'CutterCloseError'], 
			input_keys=['result'], output_keys=['cutterStatus', 'result'])
		
	def execute(self, userdata):
		'''
		Cut the stem by closing and opening the cutter.
		
		@return 'StemCutted' when the cutting was successful, 'CutterError' otherwise
		'''
		rospy.loginfo('Executing state Cut')
		if self.setIO(CUTTER_OUT, CUTTER_CLOSE, CUTTER_IN, CUTTER_CLOSE):
			if self.setIO(CUTTER_OUT, CUTTER_OPEN, CUTTER_IN, CUTTER_OPEN):
				userdata.result.cut = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
				return 'StemCutted'
			else:
				userdata.result.cut = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the cutter')
				return 'CutterOpenError'
		userdata.cutterStatus = 'CUTT_ERR'
		userdata.result.cut = HarvestStatus(success = HarvestStatus.ERROR, message = 'Cannot close the cutter')
		return 'CutterCloseError'

class MoveToDropoff(smach.State):
	'''
	State representing the arm movement towards the dropoff location
	'''
	def __init__(self, moveArmTo):
		self.moveArmTo = moveArmTo
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], 
			input_keys=['result', 'data'], output_keys=['result'])

	def execute(self, userdata):
		'''
		Move the arm to the dropoff location provided by the input data.
		
		@return 'MoveOK' when the move was successful, 'MoveError' otherwise
		'''
		rospy.loginfo('Executing state MoveToDropoff')
		(plan, move) = self.moveArmTo(userdata.data.dropLocation)
		if not plan:
			userdata.result.moveToDropoff = HarvestStatus(success = HarvestStatus.FATAL, message = 'Error planning the movement')
			return 'MoveError'			
		if move:
			userdata.result.moveToDropoff = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			return 'MoveOK'
		else:
			userdata.result.moveToDropoff = HarvestStatus(success = HarvestStatus.FATAL, message = 'Error moving to the target')
			return 'MoveError'

class Release(smach.State):
	'''
	State representing the releasing of the produce from the gripper.
	'''
	def __init__(self, setIO):
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['ReleasedAll', 'GripperError', 'VacuumError', 'CutterError', 'ReleaseError'], 
			input_keys=['systemStatus', 'result'], output_keys=['result'])

	def execute(self, userdata):
		'''
		Releases the cucumber by opening the gripper and disabling the vacuum.
		When the system is in an error state, not all those actions will be performed.
		
		@return 'ReleasedAll' when all went well, 'ReleaseError' when releasing the produce did not succeed,
		one of ['GripperError', 'VacuumError', 'CutterError'] when releasing went well after the system war in the corresponding error state.
		'''
		rospy.loginfo('Executing state Release')
		# TODO: Swap these according to the dropping procedure, 
		# this order seems fine for horizontal dropping in a crate.
		if (userdata.systemStatus is 'OK'):
			if (self.setIO(GRIPPER_OUT, GRIPPER_OPEN, GRIPPER_IN, GRIPPER_OPEN) and
				self.setIO(VACUUM_OUT, VACUUM_OFF, VACUUM_IN, VACUUM_OFF)):
				userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
				return 'ReleasedAll'
			else:
				userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot release the produce')
				
			
		elif userdata.systemStatus is 'GRABB_ERR':
			if self.setIO(GRIPPER_OUT, GRIPPER_OPEN, GRIPPER_IN, GRIPPER_OPEN):
				userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the gripper')
				return 'GripperError'
			else:
				userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			
		elif userdata.systemStatus is 'VACU_ERR':
			if (self.setIO(VACUUM_OUT, VACUUM_OFF, VACUUM_IN, VACUUM_OFF) and
				self.setIO(GRIPPER_OUT, GRIPPER_OPEN, GRIPPER_IN, GRIPPER_OPEN)):
				userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')	
				return 'VacuumError'
			else:
				userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot release the produce')
			
		elif userdata.systemStatus is 'CUTT_ERR':
			userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			return 'CutterError'
		
		return 'ReleaseError'
		

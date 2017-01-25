#!/usr/bin/env python

import rospy
import smach
import smach_ros
import tf
import math, copy
from geometry_msgs.msg import Quaternion, Pose, Point
from cucumber_msgs.msg import HarvestStatus
import sceneObjects as sceneObj

from enums import MoveStatus

# Output pins
VACUUM_OUT = 0
CUTTER_OUT = 1
GRIPPER_OUT = 2

# Input pins
VACUUM_IN = 0
CUTTER_IN_CLOSED = 1
GRIPPER_IN_CLOSED = 2
CUTTER_IN_OPENED = 3
GRIPPER_IN_OPENED = 4

GRIPPER_OPEN = True
GRIPPER_CLOSE = False
CUTTER_CLOSE = True
CUTTER_OPEN = False
VACUUM_ON = True
VACUUM_OFF = False

ACTIONS = {
	# "Task" : (OutputPort, OutputSignal, InputPort, ExpectedInput),
	"TurnVacuumOn" : (VACUUM_IN, VACUUM_ON, VACUUM_OUT, True),
	"TurnVacuumOff" : (VACUUM_IN, VACUUM_OFF, VACUUM_OUT, False),
	"OpenCutter" : (CUTTER_OUT, CUTTER_OPEN, CUTTER_IN_OPENED, True),
	"CloseCutter" : (CUTTER_OUT, CUTTER_CLOSE, CUTTER_IN_CLOSED, True),
	"CloseGripper" : (GRIPPER_OUT, GRIPPER_CLOSE, GRIPPER_IN_CLOSED, True),
	"OpenGripper" : (GRIPPER_OUT, GRIPPER_OPEN, GRIPPER_IN_OPENED, True),
}

# Orientation of the cucumber when released
HORIZONTAL = True
VERTICAL = False

dropOrient = VERTICAL

class MoveToCucumber(smach.State):
	'''
	State representing the movement towards the cucumber.
	'''
	def __init__(self, moveArmTo, setIO, pubPlanningScene, get_planning_scene):
		self.setIO = setIO
		self.moveArmTo = moveArmTo
		self.pubPlanningScene = pubPlanningScene
		self.get_planning_scene = get_planning_scene
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError', 'GripperError'], input_keys=['data', 'result'], output_keys=['result'])

	def execute(self, userdata):
		'''
		Move the arm to the position of the cucumber and exit this state when reached
		
		@return 'MoveOK' when the move was successful, 'MoveError' otherwise
		'''
		rospy.loginfo('Executing state MoveToCucumber')
		q = tf.transformations.quaternion_from_euler(0, 0, .5*math.pi)
		p = userdata.data.cucumber.stem_position
		position = Point(p.x, p.z, p.y)
		pose = Pose(position, Quaternion(q[0], q[1], q[2], q[3]))
		pose.position.y = pose.position.y - 0.1 - 0.3
		pose.position.z = pose.position.z + 0.05
		
		res = self.moveArmTo(pose)
		if res is MoveStatus.PLAN_ERROR:
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error planning the movement to the grasp start position')
			return 'MoveError'
		elif res is MoveStatus.MOVE_OK:
			if (self.setIO(*ACTIONS["OpenGripper"])):
				userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Successfully opened the gripper')
			else:
				userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the gripper')
				return 'GripperError'
			
			sceneObj.toggleOctomap(True, self.pubPlanningScene, self.get_planning_scene)
			pose.position.y = pose.position.y + 0.1 + 0.05
			res = self.moveArmTo(pose)
			if res is MoveStatus.PLAN_ERROR:
				userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error planning grasping movement towards the produce')
				return 'MoveError'
			elif res is MoveStatus.MOVE_OK:
				userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
				return 'MoveOK'
			else:
				userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error moving straight to the target')
				return 'MoveError'
		else:
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error moving to the grasp start position')
			return 'MoveError'

class CloseGripper(smach.State):
	'''
	State representing the the closing of the gripper.
	'''
	def __init__(self, moveArmTo, group, setIO):
		self.moveArmTo = moveArmTo
		self.group = group
		self.setIO = setIO
		smach.State.__init__(self, outcomes=['GripperClosed', 'GripperFail', 'GripperError', 'RepositionFailed'], 
			input_keys=['result'], output_keys=['gripperStatus', 'result'])
		self.tries = 1
		self.maxTries = 3

	def execute(self, userdata):
		'''
		Close the gripper and check if this is actualy done.
		
		@return 'GripperClosed' when successful, 'GripperError' otherwise
		'''
		rospy.loginfo('Executing state CloseGripper')
		if self.setIO(*ACTIONS["CloseGripper"]):
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			self.tries = 1
			return 'GripperClosed'
		else:
			if self.tries < self.maxTries:
				self.tries += 1
				return 'GripperFail'
			self.tries = 1
			
			
			pose = self.group.get_current_pose().pose
			pose.position.y = pose.position.y - 0.1 # Move 10cm back
			if (self.setIO(*ACTIONS["OpenGripper"])):
				userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Successfully opened the gripper')
				if self.moveArmTo(pose) is MoveStatus.MOVE_OK:
					return 'GripperError'
				else:
					return 'RepositionFailed'

			else:
				userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the gripper')
				return 'GripperError'
			
			
			userdata.result.grip = HarvestStatus(success = HarvestStatus.ERROR, message = 'Cannot grab the cucumber')
			userdata.gripperStatus = 'GRAB_ERR'
			return 'GripperError'

class RepositionGripper(smach.State):
	'''
	State representing the repositioning of the gripper when the gripping action fails.
	'''
	def __init__(self, moveArmTo, group, setIO):
		self.setIO = setIO
		self.moveArmTo = moveArmTo
		self.group = group
		smach.State.__init__(self, outcomes=['Repositioned', 'RepositionFailed', 'GripperError'], 
			input_keys=['result'], output_keys=['gripperStatus', 'result'])

	def execute(self, userdata):
		'''
		Mov the gripper away from the produce.
		
		@return 'Repositioned' when successful, 'RepositionFailed' otherwise
		'''
		rospy.loginfo('Executing state RepositionGripper')
		
		pose = self.group.get_current_pose().pose
		pose.position.y = pose.position.y - 0.1 # Move 10cm back
		if (self.setIO(*ACTIONS["OpenGripper"])):
			userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Successfully opened the gripper')
			
			if self.moveArmTo(pose) is MoveStatus.MOVE_OK:
				return 'Repositioned'
			else:
				return 'RepositionFailed'
		else:
			userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the gripper')
			return 'GripperError'

class VacuumGrip(smach.State):
	'''
	State representing the gripping of the cucumber using the vacuum suction cup.
	'''
	def __init__(self, setIO, moveArmTo, group):
		self.setIO = setIO
		self.moveArmTo = moveArmTo
		self.group = group
		smach.State.__init__(self, outcomes=['VacuumCreated', 'VacuumFail', 'VacuumError', 'MoveError'], 
			input_keys=['result'], output_keys=['vacuumStatus', 'result'])
		self.tries = 1
		self.maxTries = 3

	def execute(self, userdata):
		'''
		Enable the vacuum and make sure it is correctly initiated.
		
		@return 'VacuumCreated' when the vacuum is created succesfully, 'VacuumError' otherwise
		'''
		rospy.loginfo('Executing state VacuumGrip')
		if self.setIO(*ACTIONS["TurnVacuumOn"]):
			userdata.result.grip = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
			self.tries = 1
			return 'VacuumCreated'
		else:
			if self.tries < self.maxTries:
				self.tries += 1
				return 'VacuumFail'
			self.tries = 1
				
			userdata.result.grip = HarvestStatus(success = HarvestStatus.ERROR, message = 'Cannot create vacuum')
			userdata.vacuumStatus = 'VACU_ERR'
			
			pose = self.group.get_current_pose().pose
			pose.position.y = pose.position.y - 0.1 # Move 10cm back
			if not self.moveArmTo(pose) is MoveStatus.MOVE_OK:
				return 'MoveError'
			
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
		
		startOrientation = copy.copy(self.group.get_current_pose().pose.orientation)
		print(startOrientation)
		pose = self.rotate(self.group.get_current_pose().pose, -.25) # ~15 degrees
	
		if self.moveArmTo(pose) is MoveStatus.MOVE_OK:
			pose.orientation = startOrientation
			if self.moveArmTo(pose) is MoveStatus.MOVE_OK:
				print(startOrientation)
				return 'TiltOK'
		return 'TiltError'
	
	def rotate(self, pose, angle):
		'''
		Calculate the new pose after applying a rotation to the old pose.
		
		@param pose: The current pose
		@param angle: The angle in radians that it must turn.
		@return: The new pose
		'''
		o = pose.orientation
		orientation = (o.x, o.y, o.z, o.w)
		
		rotation = tf.transformations.quaternion_from_euler(0, angle, 0)
		o = tf.transformations.quaternion_multiply(orientation, rotation)
		
		pose.orientation.x = o[0]
		pose.orientation.y = o[1]
		pose.orientation.z = o[2]
		pose.orientation.w = o[3]
		
		return pose
		
class Cut(smach.State):
	'''
	State representing the cutting of the stem of the cucumber.
	'''
	def __init__(self, setIO, acoPub, group):
		self.setIO = setIO
		self.acoPub = acoPub
		self.group = group
		smach.State.__init__(self, outcomes=['StemCutted', 'CutterOpenError', 'CutterCloseError'], 
			input_keys=['result', 'data'], output_keys=['cutterStatus', 'result'])
		
	def execute(self, userdata):
		'''
		Cut the stem by closing and opening the cutter.
		
		@return 'StemCutted' when the cutting was successful, 'CutterError' otherwise
		'''
		rospy.loginfo('Executing state Cut')
		if self.setIO(*ACTIONS["CloseCutter"]):
			if self.setIO(*ACTIONS["OpenCutter"]):
				userdata.result.cut = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
				self.acoPub.publish(sceneObj.cucumber(userdata.data.cucumber, self.group.get_current_pose().pose))
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
	def __init__(self, moveArmTo, group, pubPlanningScene, get_planning_scene):
		self.moveArmTo = moveArmTo
		self.group = group
		self.pubPlanningScene = pubPlanningScene
		self.get_planning_scene = get_planning_scene
		smach.State.__init__(self, outcomes=['MoveOK', 'MoveError'], 
			input_keys=['result', 'data'], output_keys=['result'])

	def execute(self, userdata):
		'''
		Move the arm to the dropoff location provided by the input data.
		
		@return 'MoveOK' when the move was successful, 'MoveError' otherwise
		'''
		rospy.loginfo('Executing state MoveToDropoff')
		pose = self.group.get_current_pose().pose
		pose.position.y -= 0.1
		
		res = self.moveArmTo(pose)
		if res is MoveStatus.PLAN_ERROR:
			userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error planning the move out of the plant')
			return 'MoveError'
		elif res is MoveStatus.MOVE_OK:
			sceneObj.toggleOctomap(False, self.pubPlanningScene, self.get_planning_scene)
			pose = userdata.data.dropLocation
			res = self.moveArmTo(pose)
			if res is MoveStatus.PLAN_ERROR:
				userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error planning the move to the dropoff location')
				return 'MoveError'
			elif res is MoveStatus.MOVE_OK:
				userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
				return 'MoveOK'
			else:
				userdata.result.moveToTarget = HarvestStatus(success = HarvestStatus.ERROR, message = 'Error moving to dropoff')
				return 'MoveError'

class Release(smach.State):
	'''
	State representing the releasing of the produce from the gripper.
	'''
	def __init__(self, setIO, acoPub):
		self.setIO = setIO
		self.acoPub = acoPub
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
		if (userdata.systemStatus is 'OK'):
                        if (dropOrient):                # Horizontal orientation when released
                                if self.setIO(*ACTIONS["OpenGripper"]):
                                        if (self.setIO(*ACTIONS["TurnVacuumOff"])):
                                                userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
                                                self.acoPub.publish(sceneObj.remCucumber())
                                                return 'ReleasedAll'
                                        else:
                                                userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot stop the suction, thus cannot release the produce')
                                else:
                                        userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the gripper')
                        else:                           # Vertical orientation when released
                                if self.setIO(*ACTIONS["TurnVacuumOff"]):
                                        if (self.setIO(*ACTIONS["OpenGripper"])):
                                                userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
                                                self.acoPub.publish(sceneObj.remCucumber())
                                                return 'ReleasedAll'
                                        else:
                                                userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the gripper')
                                else:
                                        userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot stop the suction, thus cannot release the produce')
				
			
		elif userdata.systemStatus is 'GRAB_ERR':
			if self.setIO(*ACTIONS["OpenGripper"]):
				userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')
				return 'GripperError'
			else:
				userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the gripper')
			
		elif userdata.systemStatus is 'VACU_ERR':
			if (self.setIO(*ACTIONS["TurnVacuumOff"]) and
				self.setIO(*ACTIONS["OpenGripper"])):
				userdata.result.release = HarvestStatus(success = HarvestStatus.OK, message = 'Success')	
				return 'VacuumError'
			else:
				userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot release the produce')
			
		elif userdata.systemStatus is 'CUTT_ERR':
				if (self.setIO(*ACTIONS["OpenCutter"])):
					userdata.result.release = HarvestStatus(success = HarvestStatus.FATAL, message = 'Cannot open the cutter')
					return 'CutterError'
		return 'ReleaseError'

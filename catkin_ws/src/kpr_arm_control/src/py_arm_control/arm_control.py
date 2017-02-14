#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg
import moveit_commander
import smach
import smach_ros
from cucumber_msgs.srv import HarvestAction, HarvestActionResponse
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetIORequest

from enums import MoveStatus
import states as state
import sceneObjects as sceneObj

success = {
	"ERROR": HarvestActionResponse.ERROR,
	"OK": HarvestActionResponse.OK,
	"GRAB_ERR": HarvestActionResponse.GRAB_ERR,
	"VACU_ERR": HarvestActionResponse.VACU_ERR,
	"CUTT_ERR": HarvestActionResponse.CUTT_ERR,
	"MOVE_ERR": HarvestActionResponse.MOVE_ERR
}
ATTEMPTS = 10

analogPinStates = [False] * 2
digitalPinStates = [False] * 10

def createStateMachine():
	'''
	Create a fresh state machine.

	@return a state machine object
	'''
	global group, aco_pub, pubPlanningScene, get_planning_scene
	sm = smach.StateMachine(outcomes=['OK', 'GRAB_ERR', 'VACU_ERR', 'CUTT_ERR', 'MOVE_ERR', 'ERROR'])
	sm.userdata.status = 'OK'
	sm.userdata.result = HarvestActionResponse()

	with sm:											
		smach.StateMachine.add('MoveToCucumber', state.MoveToCucumber(moveArmTo, writeWithDigitalFeedback, pubPlanningScene, get_planning_scene), 
							   transitions={'MoveOK':'CloseGripper',
											'MoveError':'MOVE_ERR',
											'GripperError': 'GRAB_ERR'},
							   remapping={	'data':'request',
											'result':'result'})
											
		smach.StateMachine.add('CloseGripper', state.CloseGripper(moveArmTo, group, writeWithDigitalFeedback), 
							   transitions={'GripperClosed':'VacuumGrip',
											'GripperFail':'RepositionGripper',
											'GripperError':'Release',
											'RepositionFailed' : 'MOVE_ERR'},
							   remapping={	'gripperStatus':'status',
											'result':'result'})
											
		smach.StateMachine.add('RepositionGripper', state.RepositionGripper(moveArmTo, group, writeWithDigitalFeedback), 
							   transitions={'Repositioned':'MoveToCucumber',
											'RepositionFailed':'CloseGripper',
											'GripperError':'GRAB_ERR'})
											
		smach.StateMachine.add('VacuumGrip', state.VacuumGrip(writeWithDigitalFeedback, moveArmTo, group), 
							   transitions={'VacuumCreated':'Cut',
											'VacuumFail':'Tilt',
											'VacuumError':'Release',
											'MoveError':'MOVE_ERR'},
							   remapping={	'vacuumStatus':'status',
											'result':'result'})
											
		smach.StateMachine.add('Tilt', state.Tilt(moveArmTo, group), 
							   transitions={'TiltOK':'VacuumGrip',
											'TiltError':'VacuumGrip'})
											
		smach.StateMachine.add('Cut', state.Cut(writeWithDigitalFeedback, aco_pub, group), 
							   transitions={'StemCutted':'MoveToDropoff',
											'CutterOpenError':'ERROR',
											'CutterCloseError':'Release'},
							   remapping={	'data':'request',
											'cutterStatus':'status',
											'result':'result'})
											
		smach.StateMachine.add('MoveToDropoff', state.MoveToDropoff(moveArmTo, group, pubPlanningScene, get_planning_scene), 
							   transitions={'MoveOK':'Release',
											'MoveError':'ERROR'},
							   remapping={	'data':'request',
											'result':'result'})
											
		smach.StateMachine.add('Release', state.Release(writeWithDigitalFeedback, aco_pub), 
							   transitions={'ReleasedAll':'OK',
											'GripperError':'GRAB_ERR',
											'VacuumError':'VACU_ERR',
											'CutterError':'CUTT_ERR',
											'ReleaseError':'ERROR'},
							   remapping={'systemStatus':'status',
											'result':'result'})
	return sm

def moveArmTo(pose_target):
	'''
	Plan the movement to the specified position and move the arm.

	@param pose_target: The Pose of the goal position
	@return an enum value from MoveStatus corresponding to the success
	'''
	global group, robot
	group.set_start_state_to_current_state()
	group.clear_pose_targets()
	group.set_pose_target(pose_target)
	if not group.plan():
		print pose_target.position
		return MoveStatus.PLAN_ERROR
	if group.go(wait=True):
		return MoveStatus.MOVE_OK
	else:
		return MoveStatus.MOVE_ERROR
	
def setJointPositions(joint_states):
	'''
	Plan the movement to the specified joint positions and move the arm.

	@param joint_states: The joint states of the goal position
	@return an enum value from MoveStatus corresponding to the success
	'''
	global group, robot
	group.set_start_state_to_current_state()
	group.clear_pose_targets()
	
	group.set_joint_value_target(joint_states)

	if not group.plan():
		return MoveStatus.PLAN_ERROR
	if group.go(wait=True):
		return MoveStatus.MOVE_OK
	else:
		return MoveStatus.MOVE_ERROR
	 
def getCucumberCallback (req):
	'''
	Callback for the cucumber targets.
	Starts the state machine with the provided data

	@param req: The HarvestAction request sent
	@return A HarvestActionResponse with the success codes
	'''
	global stateMachine
	stateMachine.userdata.request = req
	outcome = stateMachine.execute()
	response = stateMachine.userdata.result
	response.status = success[outcome]
	return response

def addSceneObjects(aco_publisher):
	'''
	Add the table and endeffector to the planning scene.

	@param aco_publisher: The publisher for the scene objects
	'''
	global group
	aco_publisher.publish(sceneObj.table())
	aco_publisher.publish(sceneObj.roof())
	
def setupMoveIt():
	'''
	Initialize the moveIt! framework

	@return A triplet containing the robot- and group commander and the planning scene
	'''
	rospy.loginfo("Waiting for moveit service")
	rospy.wait_for_service('/plan_kinematic_path')
	rospy.loginfo("Moveit service connected")
	
	moveit_commander.roscpp_initialize(sys.argv)
	group = moveit_commander.MoveGroupCommander("manipulator")
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	return (robot, scene, group)

def IOStatesCallback(msg):
	'''
	Callback method for receiving io states from the arm.
	Stores the received states.
	'''
	global analogPinStates, digitalPinStates
	for pin in msg.digital_in_states:
		digitalPinStates[pin.pin] = pin.state
		
	for pin in msg.analog_in_states:
		analogPinStates[pin.pin] = pin.state
	
def setIO(pin, value):
	'''
	Set the Value of a specified IO pin.
	Note that it only writes to digital pins as we currently are not using the analog pins.

	@param pin: The pin to write to
	@param value: The value that is written to the pin
	@return True when the value is set correctly, False otherwise
	'''
	try:
		set_io = rospy.ServiceProxy('set_io', SetIO)
		return set_io(SetIORequest.FUN_SET_DIGITAL_OUT, pin, value)
	except rospy.ServiceException, e:
		rospy.logwarn("Service call failed: %s", e)
		return False

def getAnalog(pin):
	'''
	Get the value of the specified analog pin

	@param pin: The pin to get the value of
	@return The analog value of the pin
	'''
	global analogPinStates
	return analogPinStates[pin]

def getDigital(pin):
	'''
	Get the value of the specified digital pin

	@param pin: The pin to get the value of
	@return The digital value of the pin (True/False)
	'''
	global digitalPinStates
	return digitalPinStates[pin]
	
def setupIO():
	'''
	Setup the IO subscriber and the setIO service.
	@return The subscriber to the io_states channel
	'''
	rospy.wait_for_service('set_io')
	io_states_sub = rospy.Subscriber("io_states", IOStates, IOStatesCallback);
	return io_states_sub

def writeWithDigitalFeedback(outPin, value, inPin, expected):
	'''
	Write a value to the specified pin and check if another pin reads an expected value.
	This last check is performed multiple times to account for slow changing values.

	@param outPin: The pin to write the value to
	@param value: The value to write to the inPin
	@param inPin: The pin to read from
	@param expected: The expected value on the inPin
	@return True when writing succeeds and the expected value is read, False otherwise
	'''
	if not setIO(outPin, value):
		return False
	
	for i in range(ATTEMPTS):
		if getDigital(inPin) is expected:
			return True
		rospy.sleep(.1)
	return False

def writeWithAnalogFeedback(outPin, value, inPin, expected, delta):
	'''
	Write a value to the specified pin and check if another pin reads an expected value.
	This last check is performed multiple times to account for slow changing values.

	@param outPin: The pin to write the value to
	@param value: The value to write to the inPin
	@param inPin: The pin to read from
	@param expected: The expected value on the inPin
	@param delta: The maximum difference between the read and expected value to be considered equal
	@return True when writing succeeds and the expected value is read, False otherwise
	'''
	if not setIO(outPin, value):
		return False
	
	for i in range(ATTEMPTS):
		if abs(getAnalog(inPin) - expected) < delta:
			return True
		rospy.sleep(.1)
	return False

if __name__ == '__main__':
	'''
	Main method of arm_control.py.
	Starts moveIt! and registers subscribers/publishers.
	'''
	rospy.init_node('ArmControl')
	s = rospy.Service('target/cucumber', HarvestAction, getCucumberCallback)
	aco_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject, queue_size=10)
	pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene)
	rospy.wait_for_service('/get_planning_scene', 10.0)
	get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
	(robot, scene, group) = setupMoveIt()
	io_states_sub = setupIO()
	stateMachine = createStateMachine()
	rospy.loginfo("Started")
	rospy.sleep(2)
	addSceneObjects(aco_pub)
	rospy.spin()
	moveit_commander.roscpp_shutdown()
	rospy.loginfo("Stopped")

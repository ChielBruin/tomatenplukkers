#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg
import moveit_commander
import smach
import smach_ros
from cucumber_msgs.srv import HarvestAction, HarvestActionResponse
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetIORequest

import states as state
import sceneObjects as sceneObj

success = {
	"OK": HarvestActionResponse.OK,
	"GRAB_ERR": HarvestActionResponse.GRAB_ERR,
	"CUTT_ERR": HarvestActionResponse.CUTT_ERR,
	"DROP_ERR": HarvestActionResponse.DROP_ERR,
	"MOVE_ERR": HarvestActionResponse.MOVE_ERR
}
startingPosition = Pose()
ATTEMPTS = 10

analogPinStates = [False] * 10
digitalPinStates = [False] * 10

def createStateMachine():
	global startingPosition
	sm = smach.StateMachine(outcomes=['OK', 'GRAB_ERR', 'CUTT_ERR', 'DROP_ERR', 'MOVE_ERR'])
	sm.userdata.startingPosition = startingPosition

	with sm:											
		smach.StateMachine.add('MoveToCucumber', state.MoveToCucumber(moveArmTo), 
							   transitions={'MoveOK':'CloseGripper',
											'MoveError':'MOVE_ERR'},
							   remapping={	'data':'request'})
											
		smach.StateMachine.add('CloseGripper', state.CloseGripper(writeWithDigitalFeedback), 
							   transitions={'GripperClosed':'VacuumGrip',
											'GripperError':'GRAB_ERR'})
											
		smach.StateMachine.add('VacuumGrip', state.VacuumGrip(writeWithDigitalFeedback), 
							   transitions={'VacuumCreated':'Cut',
											'VacuumError':'GRAB_ERR'})
											
		smach.StateMachine.add('Cut', state.Cut(writeWithDigitalFeedback), 
							   transitions={'StemCutted':'MoveToDropoff',
											'CutterError':'CUTT_ERR'})
											
		smach.StateMachine.add('MoveToDropoff', state.MoveToDropoff(moveArmTo), 
							   transitions={'MoveOK':'OpenGripper',
											'MoveError':'MOVE_ERR'},
							   remapping={	'data':'request'})
											
		smach.StateMachine.add('OpenGripper', state.OpenGripper(writeWithDigitalFeedback), 
							   transitions={'GripperOpened':'MoveToStart',
											'GripperError':'GRAB_ERR'})
											
		smach.StateMachine.add('MoveToStart', state.MoveToStart(moveArmTo), 
							   transitions={'MoveOK':'OK',
											'MoveError':'MOVE_ERR'},
							   remapping={	'start':'startingPosition'})
	return sm

def moveArmTo(pose_target):
	global group, robot
	# group.set_start_state(robot.get_current_state())
	group.set_start_state_to_current_state()
	group.clear_pose_targets()
	group.set_pose_target(pose_target)
	if not group.plan():
		return False
	return group.go(wait=True)
	 
def getCucumberCallback (req):
	sm = createStateMachine()
	sm.userdata.request = req
	outcome = sm.execute()	
	return HarvestActionResponse(success[outcome])

def addSceneObjects(aco_publisher):
	global group
	aco_publisher.publish(sceneObj.table())
	aco_publisher.publish(sceneObj.endEffector(group))
	
def setupMoveIt():
	moveit_commander.roscpp_initialize(sys.argv)
	group = moveit_commander.MoveGroupCommander("manipulator")
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	return (robot, scene, group)

def IOStatesCallback(msg):
	global analogPinStates, digitalPinStates
	for pin in msg.digital_in_states:
		digitalPinStates[pin.pin] = pin.state
		
	for pin in msg.analog_in_states:
		analogPinStates[pin.pin] = pin.state
	
def setIO(pin, value):
	try:
		#set_io = rospy.ServiceProxy('set_io', SetIO)
		set_io = rospy.ServiceProxy('set_io_testing', SetIO)
		return set_io(SetIORequest.FUN_SET_DIGITAL_OUT, pin, value)
	except rospy.ServiceException, e:
		rospy.logwarn("Service call failed: %s", e)
		return False

def getAnalog(pin):
	global analogPinStates
	return analogPinStates[pin]

def getDigital(pin):
	global digitalPinStates
	return digitalPinStates[pin]
	
def setupIO():
	#rospy.wait_for_service('set_io')
	rospy.wait_for_service('set_io_testing')
	#io_states_sub = rospy.Subscriber("io_states", IOStates, IOStatesCallback);
	io_states_sub = rospy.Subscriber("io_states_testing", IOStates, IOStatesCallback);
	return io_states_sub

def writeWithDigitalFeedback(outPin, value, inPin, expected):
	if not setIO(outPin, value):
		return False
	
	for i in range(ATTEMPTS):
		if getDigital(inPin) is expected:
			return True
		rospy.sleep(.1)
	return False

def writeWithAnalogFeedback(outPin, value, inPin, expected, delta):
	if not setIO(outPin, value):
		return False
	
	for i in range(ATTEMPTS):
		if abs(getAnalog(inPin) - expected) < delta:
			return True
		rospy.sleep(.1)
	return False
	
if __name__ == '__main__':
	rospy.init_node('ArmControl')
	s = rospy.Service('target/cucumber', HarvestAction, getCucumberCallback)
	aco_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject, queue_size=10)
	(robot, scene, group) = setupMoveIt()
	io_states_sub = setupIO()
	startingPosition = group.get_current_pose()
	rospy.loginfo("Started")
	addSceneObjects(aco_pub)
	rospy.spin()
	moveit_commander.roscpp_shutdown()
	rospy.loginfo("Stopped")

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

def createStateMachine():
	global startingPosition
	sm = smach.StateMachine(outcomes=['OK', 'GRAB_ERR', 'CUTT_ERR', 'DROP_ERR', 'MOVE_ERR'])
	sm.userdata.startingPosition = startingPosition

	with sm:											
		smach.StateMachine.add('MoveToCucumber', state.MoveToCucumber(moveArmTo), 
							   transitions={'MoveOK':'CloseGripper',
											'MoveError':'MOVE_ERR'},
							   remapping={	'data':'request'})
											
		smach.StateMachine.add('CloseGripper', state.CloseGripper(), 
							   transitions={'GripperClosed':'VacuumGrip',
											'GripperError':'GRAB_ERR'})
											
		smach.StateMachine.add('VacuumGrip', state.VacuumGrip(), 
							   transitions={'VacuumCreated':'Cut',
											'VacuumError':'GRAB_ERR'})
											
		smach.StateMachine.add('Cut', state.Cut(), 
							   transitions={'StemCutted':'MoveToDropoff',
											'CutterError':'CUTT_ERR'})
											
		smach.StateMachine.add('MoveToDropoff', state.MoveToDropoff(moveArmTo), 
							   transitions={'MoveOK':'OpenGripper',
											'MoveError':'MOVE_ERR'},
							   remapping={	'data':'request'})
											
		smach.StateMachine.add('OpenGripper', state.OpenGripper(), 
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
	group = None;
	while group is None:
		try:
			group = moveit_commander.MoveGroupCommander("manipulator")
		except RuntimeError:
			rospy.logwarn("Could not contact the move group.")
			import pprint
			pprint.pprint(group)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	return (robot, scene, group)

if __name__ == '__main__':
	rospy.init_node('ArmControl')
	s = rospy.Service('target/cucumber', HarvestAction, getCucumberCallback)
	aco_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject, queue_size=10)
	(robot, scene, group) = setupMoveIt()
	startingPosition = group.get_current_pose()
	rospy.loginfo("Started")
	addSceneObjects(aco_pub)
	rospy.spin()
	moveit_commander.roscpp_shutdown()
	rospy.loginfo("Stopped")

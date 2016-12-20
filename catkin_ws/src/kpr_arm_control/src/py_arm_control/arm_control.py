#!/usr/bin/env python

import rospy
import smach
import smach_ros
from cucumber_msgs.srv import HarvestAction, HarvestActionResponse

import states as state

success = {
	"OK": HarvestActionResponse.OK,
	"GRAB_ERR": HarvestActionResponse.GRAB_ERR,
	"CUTT_ERR": HarvestActionResponse.CUTT_ERR,
	"DROP_ERR": HarvestActionResponse.DROP_ERR,
	"MOVE_ERR": HarvestActionResponse.MOVE_ERR
}

def createStateMachine():
	sm = smach.StateMachine(outcomes=['OK', 'GRAB_ERR', 'CUTT_ERR', 'DROP_ERR', 'MOVE_ERR'])

	with sm:
		smach.StateMachine.add('CreatePath', state.CreatePath(), 
							   transitions={'PathFound':'MoveToCucumber', 
											'NoPath':'MOVE_ERR'})
											
		smach.StateMachine.add('MoveToCucumber', state.MoveToCucumber(), 
							   transitions={'MoveOK':'CloseGripper',
											'MoveError':'MOVE_ERR'})
											
		smach.StateMachine.add('CloseGripper', state.CloseGripper(), 
							   transitions={'GripperClosed':'VacuumGrip',
											'GripperError':'GRAB_ERR'})
											
		smach.StateMachine.add('VacuumGrip', state.VacuumGrip(), 
							   transitions={'VacuumCreated':'Cut',
											'VacuumError':'GRAB_ERR'})
											
		smach.StateMachine.add('Cut', state.Cut(), 
							   transitions={'StemCutted':'MoveToDropoff',
											'CutterError':'CUTT_ERR'})
											
		smach.StateMachine.add('MoveToDropoff', state.MoveToDropoff(), 
							   transitions={'MoveOK':'OpenGripper',
											'MoveError':'MOVE_ERR'})
											
		smach.StateMachine.add('OpenGripper', state.OpenGripper(), 
							   transitions={'GripperOpened':'MoveToStart',
											'GripperError':'GRAB_ERR'})
											
		smach.StateMachine.add('MoveToStart', state.MoveToStart(), 
							   transitions={'MoveOK':'OK',
											'MoveError':'MOVE_ERR'})
	return sm
	
def getCucumberCallback (req):
	sm = createStateMachine()
	outcome = sm.execute()	
	return HarvestActionResponse(success[outcome])

# main
def main():
	rospy.init_node('ArmControl')
	s = rospy.Service('target/cucumber', HarvestAction, getCucumberCallback)
	
	ROS_INFO("Started");
	rospy.spin()
	ROS_INFO("Stopped");

if __name__ == '__main__':
    main()

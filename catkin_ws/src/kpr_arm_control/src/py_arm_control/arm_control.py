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
	sm = smach.StateMachine(outcomes=['OK', 'MOVE_ERR'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add('FOO', state.Foo(), 
							   transitions={'outcome1':'BAR', 
											'outcome2':'OK'})
		smach.StateMachine.add('BAR', state.Bar(), 
							   transitions={'outcome2':'FOO'})
	return sm
	
def getCucumberCallback (req):
	sm = createStateMachine()
	outcome = sm.execute()	
	return HarvestActionResponse(success[outcome])

# main
def main():
	rospy.init_node('ArmControl')
	s = rospy.Service('target/cucumber', HarvestAction, getCucumberCallback)

	sm = createStateMachine()
	rospy.spin()

if __name__ == '__main__':
    main()

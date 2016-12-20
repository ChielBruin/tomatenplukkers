#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.counter = 0

	def execute(self, userdata):
		rospy.loginfo('Executing state FOO')
		if self.counter < 3:
			self.counter += 1
			return 'outcome1'
		else:
			return 'outcome2'


# define state Bar
class Bar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome2'])

	def execute(self, userdata):
		rospy.loginfo('Executing state BAR')
		return 'outcome2'

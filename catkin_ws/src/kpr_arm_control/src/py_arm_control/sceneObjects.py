#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

def table():
	'''
	Create the table and back plate that is connected to the robot base.

	@return An AttachedCollisionObject representing the table and back plate
	'''
	HEIGHT_OFFSET = 0.004

	table = AttachedCollisionObject()
	table.link_name = "base_link"
	table.object.header.frame_id = "/world"
	table.object.id = "table"

	tablePose = Pose()
	tablePose.position.z = -0.1/2 - HEIGHT_OFFSET #- 0.26
	tablePose.orientation.w = 1.0

	tableBox = SolidPrimitive()
	tableBox.type = tableBox.BOX
	tableBox.dimensions = [1.5,1,0.1]

	table.object.primitives.append(tableBox)
	table.object.primitive_poses.append(tablePose)

	backpanelPose = Pose()
	backpanelPose.position.y = -0.3
	backpanelPose.position.z = 0.5 - HEIGHT_OFFSET - 0.26
	backpanelPose.orientation.w = 1.0

	backpanelBox = SolidPrimitive()
	backpanelBox.type = backpanelBox.BOX
	backpanelBox.dimensions = [1.5,0.1,1]

	table.object.primitives.append(backpanelBox)
	table.object.primitive_poses.append(backpanelPose)
	table.object.operation = table.object.ADD

	return table
	
def roof():
	'''
	Create a roof that constrains the random movements of the robot.

	@return An AttachedCollisionObject representing the roof
	'''

	roof = AttachedCollisionObject()
	roof.link_name = "base_link"
	roof.object.header.frame_id = "world"
	roof.object.id = "roof"

	roofPose = Pose()
	roofPose.position.z = .6
	roofPose.orientation.w = 1.0

	roofBox = SolidPrimitive()
	roofBox.type = roofBox.BOX
	roofBox.dimensions = [1.5,1,0.025]

	roof.object.primitives.append(roofBox)
	roof.object.primitive_poses.append(roofPose)
	
	return roof

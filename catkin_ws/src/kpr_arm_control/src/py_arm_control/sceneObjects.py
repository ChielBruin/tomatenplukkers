#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

def table():
	HEIGHT_OFFSET = 0.004

	table = AttachedCollisionObject()
	table.link_name = "world"
	table.object.header.frame_id = "table_attach"
	table.object.id = "table"

	tablePose = Pose()
	tablePose.position.z = -0.1/2 - HEIGHT_OFFSET
	tablePose.orientation.w = 1.0

	tableBox = SolidPrimitive()
	tableBox.type = tableBox.BOX
	tableBox.dimensions = [1.5,1,0.1]
	#tableBox.dimensions[tableBox.BOX_X] = 1.5
	#tableBox.dimensions[tableBox.BOX_Y] = 1
	#tableBox.dimensions[tableBox.BOX_Z] = 0.1

	table.object.primitives.append(tableBox)
	table.object.primitive_poses.append(tablePose)

	backpanelPose = Pose()
	backpanelPose.position.y = -0.3
	backpanelPose.position.z = 0.5 - HEIGHT_OFFSET
	backpanelPose.orientation.w = 1.0

	backpanelBox = SolidPrimitive()
	backpanelBox.type = backpanelBox.BOX
	backpanelBox.dimensions = [1.5,1,0.1]
	#backpanelBox.dimensions[backpanelBox.BOX_X] = 1.5
	#backpanelBox.dimensions[backpanelBox.BOX_Y] = 0.1
	#backpanelBox.dimensions[backpanelBox.BOX_Z] = 1

	table.object.primitives.append(backpanelBox)
	table.object.primitive_poses.append(backpanelPose)
	table.object.operation = table.object.ADD

	return table

def endEffector():
	return None

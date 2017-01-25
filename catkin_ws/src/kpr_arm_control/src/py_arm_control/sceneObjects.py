#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, PlanningSceneComponents, PlanningScene
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
	
def cucumber(cucumber, pose):
	'''
	Create a cylinder that represents the gripped cucumber.

	@return An AttachedCollisionObject representing the roof
	'''

	aco = AttachedCollisionObject()
	aco.link_name = "ee_link"
	aco.object.header.frame_id = "world"
	aco.object.id = "cucumber"

	acoPose = pose
	acoPose.position.y += cucumber.width/2 + 0.15
	acoPose.position.z -= cucumber.height/2 + 0.071

	acoCyl = SolidPrimitive()
	acoCyl.type = acoCyl.CYLINDER
	acoCyl.dimensions = [cucumber.height, cucumber.width/2]

	aco.object.primitives.append(acoCyl)
	aco.object.primitive_poses.append(acoPose)
	aco.object.operation = aco.object.ADD
	
	return aco

def remCucumber():
	'''
	Remove a cylinder that represents the gripped cucumber.

	@return An AttachedCollisionObject representing the roof
	'''

	aco = AttachedCollisionObject()
	aco.object.header.frame_id = "world"
	aco.object.id = "cucumber"
	aco.object.operation = aco.object.REMOVE
	return aco

def toggleOctomap(boolean, pubPlanningScene, get_planning_scene):
	'''
	Set whether to ignore the point cloud or not.
	'''
	
	request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
	response = get_planning_scene(request)
	acm = response.scene.allowed_collision_matrix

	if not '<octomap>' in acm.default_entry_names:
		acm.default_entry_names += ['<octomap>']
		acm.default_entry_values += [boolean]
	else:
		index = acm.default_entry_names.index('<octomap>')
		acm.default_entry_values[index] = boolean

	planning_scene_diff = PlanningScene(
		is_diff=True,
		allowed_collision_matrix=acm)

	pubPlanningScene.publish(planning_scene_diff)
	rospy.sleep(1.0)

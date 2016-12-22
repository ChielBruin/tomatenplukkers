

/**
 * Adds the table the arm stands on to the planning scene, to prevent the
 * arm from breaking itself and the end effector. Also adds a backpanel
 * because the arm won't be able to navigate too far backwards because
 * another row of cucumber plants will be there, which won't be visible.
 */
moveit_msgs::AttachedCollisionObject createTable() {
	// Offset because the base continues slightly below its link.
	const float HEIGHT_OFFSET = 0.004;

	moveit_msgs::AttachedCollisionObject table;
	table.link_name = "world";
	table.object.header.frame_id = "table_attach";
	table.object.id = "table";

	geometry_msgs::Pose tablePose;
	tablePose.position.z = -0.1/2 - HEIGHT_OFFSET;
	tablePose.orientation.w = 1.0;

	shape_msgs::SolidPrimitive tableBox;
	tableBox.type = tableBox.BOX;
	tableBox.dimensions.resize(3);
	tableBox.dimensions[tableBox.BOX_X] = 1.5;
	tableBox.dimensions[tableBox.BOX_Y] = 1;
	tableBox.dimensions[tableBox.BOX_Z] = 0.1;

	table.object.primitives.push_back(tableBox);
	table.object.primitive_poses.push_back(tablePose);

	geometry_msgs::Pose backpanelPose;
	backpanelPose.position.y = -0.3;
	backpanelPose.position.z = 0.5 - HEIGHT_OFFSET;
	backpanelPose.orientation.w = 1.0;

	shape_msgs::SolidPrimitive backpanelBox;
	backpanelBox.type = backpanelBox.BOX;
	backpanelBox.dimensions.resize(3);
	backpanelBox.dimensions[backpanelBox.BOX_X] = 1.5;
	backpanelBox.dimensions[backpanelBox.BOX_Y] = 0.1;
	backpanelBox.dimensions[backpanelBox.BOX_Z] = 1;

	table.object.primitives.push_back(backpanelBox);
	table.object.primitive_poses.push_back(backpanelPose);
	table.object.operation = table.object.ADD;

	return table;
}

/**
 * Adds the temporary end effector, to prevent the arm from breaking the
 * real end effector and to be able to properly align with the cucumber.
 */
moveit_msgs::AttachedCollisionObject createEndEffector() {
	moveit_msgs::AttachedCollisionObject endEffector;
	endEffector.link_name = "ee_link";
	endEffector.object.header.frame_id = "end_effector_attach";
	endEffector.object.id = "end_effector";

	// Translates the end effector to the end of the arm.
	// Should not be necessary but currently the built-in translation breaks.
	//TODO Make sure the built-in translation is used instead of this work around.
	geometry_msgs::Pose eePose = move_group_ptr->getCurrentPose().pose;

	eePose.position.y += 0.16/2;

	shape_msgs::SolidPrimitive eeBox;
	eeBox.type = eeBox.BOX;
	eeBox.dimensions.resize(3);
	eeBox.dimensions[eeBox.BOX_X] = 0.16;
	eeBox.dimensions[eeBox.BOX_Y] = 0.075;
	eeBox.dimensions[eeBox.BOX_Z] = 0.075;

	endEffector.object.primitives.push_back(eeBox);
	endEffector.object.primitive_poses.push_back(eePose);
	endEffector.object.operation = endEffector.object.ADD;

	return endEffector;
}

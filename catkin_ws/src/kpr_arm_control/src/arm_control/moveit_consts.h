#include <memory>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#ifndef MoveitConsts_H
#define MoveitConsts_H

typedef std::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
typedef std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningSceneInterfacePtr;

namespace moveit_consts{
	static MoveGroupPtr move_group_ptr;
	static PlanningSceneInterfacePtr planning_scene_interface_ptr;
	static std::string move_group_name("manipulator");
}

#endif

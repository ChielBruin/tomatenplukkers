cmake_minimum_required(VERSION 2.8.3)
project(kpr_arm_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roslaunch
	roscpp

	cucumber_msgs
	geometry_msgs

	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_msgs

	ur5_moveit_config
	ur_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_processing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Allow c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(arm_control src/arm_control/node_arm_control.cpp)
target_link_libraries(arm_control
	${catkin_LIBRARIES}
)
add_dependencies(arm_control
	cucumber_msgs_generate_messages_cpp
)

#############
## Install ##
#############

#############
## Testing ##
#############

roslaunch_add_file_check(launch)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_processing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

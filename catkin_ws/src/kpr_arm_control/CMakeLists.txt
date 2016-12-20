cmake_minimum_required(VERSION 2.8.3)
project(kpr_arm_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roslaunch
	roscpp
	rospy
	smach
	smach_ros

	cucumber_msgs
	geometry_msgs

	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_msgs
	moveit_commander

	ur_bringup
	ur5_moveit_config
	ur_msgs
)

catkin_python_setup()
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


catkin_install_python(PROGRAMS src/py_arm_control/arm_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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

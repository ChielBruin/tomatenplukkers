cmake_minimum_required(VERSION 2.8.3)
project(kpr_arm_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	cucumber_msgs
	geometry_msgs

	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

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

# include_directories(include)
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

add_executable(gripper_control src/gripper_control/node_gripper_control.cpp)
target_link_libraries(gripper_control
	${catkin_LIBRARIES}
)
add_dependencies(gripper_control
	cucumber_msgs_generate_messages_cpp
)

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_processing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
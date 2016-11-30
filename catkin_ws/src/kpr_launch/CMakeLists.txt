cmake_minimum_required(VERSION 2.8.3)
project(kpr_launch)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roslaunch
	kpr_interface
	ros_faster_rcnn
	core
	image_processing
	arm_control
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package()

###########
## Build ##
###########

roslaunch_add_file_check(robot.launch)

#############
## Install ##
#############]

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

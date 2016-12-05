cmake_minimum_required(VERSION 2.8.3)
project(kpr_core)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	cucumber_msgs
	kpr_interface
	diagnostic_msgs
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

# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(core src/core/node_core.cpp)
target_link_libraries(core
	${catkin_LIBRARIES}
)
add_dependencies(core
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

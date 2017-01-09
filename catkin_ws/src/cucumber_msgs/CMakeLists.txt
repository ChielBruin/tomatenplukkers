cmake_minimum_required(VERSION 2.8.3)
project(cucumber_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	std_msgs
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	Cucumber.msg
	HarvestStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	HarvestAction.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Allow c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
	${catkin_INCLUDE_DIRS}
	include
)

#############
## Install ##
#############


#############
## Testing ##
#############

catkin_add_gtest(cucumberContainer-test test/test_cucumberContainer.cpp)
	target_link_libraries(cucumberContainer-test ${catkin_LIBRARIES})

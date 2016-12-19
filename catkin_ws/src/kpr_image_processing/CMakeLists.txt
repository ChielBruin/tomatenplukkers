cmake_minimum_required(VERSION 2.8.3)
project(kpr_image_processing)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	cucumber_msgs
	ros_faster_rcnn
	stereo_msgs
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

add_executable(cVis src/cVis/node_cVis.cpp)
target_link_libraries(cVis
	${catkin_LIBRARIES}
)
add_dependencies(cVis
	cucumber_msgs_generate_messages_cpp
	ros_faster_rcnn_generate_messages_cpp
)

add_executable(IdCB src/IdCB/node_IdCB.cpp)
target_link_libraries(IdCB
	${catkin_LIBRARIES}
)
add_dependencies(IdCB
	cucumber_msgs_generate_messages_cpp
)

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(IdCB-test test/TEST_IdCB.cpp)
# if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(IdCB-test ${catkin_LIBRARIES})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

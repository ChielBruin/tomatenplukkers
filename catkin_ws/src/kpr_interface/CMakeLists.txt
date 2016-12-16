cmake_minimum_required(VERSION 2.8.3)
project(kpr_interface)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	std_msgs
	diagnostic_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
	FILES
	SetSetting.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	GetSetting.srv
	GetSettings.srv
	SettingsIO.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	diagnostic_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package()

###########
## Build ##
###########

## Allow c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS src/settingsManager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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

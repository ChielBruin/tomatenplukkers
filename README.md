# Tomatenplukkers
###  *Greenhouse harvesting robot | Minor Robotics TU Delft*
[![Build Status](https://travis-ci.org/ChielBruin/tomatenplukkers.svg?branch=master)](https://travis-ci.org/ChielBruin/tomatenplukkers)

This is the GitHub repository of the robot KasPR, that is developed as part of the [minor Robotics](http://tudelftroboticsinstitute.nl/study/minor-education-programs/minor-robotics-2016-2017) of the TU Delft. KasPR is a robot that can harvest cucumbers in greenhouses. To achieve this, the robot uses stereo vision in combination with a convolutional neural network tho detect the cucumbers and a UR5 robotic arm equipped with a pneumatic end effector to harvest the produce. 

![An image of the robot](https://imgserv4.tcdn.nl/v1/f3w3fEPVsfaIGNGP_wQ47g8viZ8=/742x557/smart/http%3A%2F%2Fcdn-kiosk-api.telegraaf.nl%2F3b29ca5e-e289-11e6-b64c-630ed5071f0f.jpg)

Part of the development of this repository is the [ros_faster_rcnn](https://github.com/ChielBruin/ros_faster_rcnn) repository. This code is created as a ROS wrapper for the python implementation of Faster RCNN.

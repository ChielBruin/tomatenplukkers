#!/bin/sh
MASTER_IP=$1

export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_IP=$MASTER_IP


#!/bin/bash
source /opt/ros/melodic/setup.sh
source /home/inha/catkin_ws/devel/setup.sh
export ROS_MASTER_URI=http://192.168.101.1:11311
export ROS_HOSTNAME=192.168.101.1

roslaunch /home/inha/catkin_ws/src/niro/launch/transceiver.launch & \
roslaunch /home/inha/catkin_ws/src/niro/launch/sensor.launch



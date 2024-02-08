#!/bin/bash
source /opt/ros/melodic/setup.sh
source $HOME/catkin_ws/devel/setup.sh
export ROS_MASTER_URI=http://192.168.101.1:11311
export ROS_HOSTNAME=192.168.101.1

roslaunch $HOME/catkin_ws/src/mobinha/selfdrive/manager/sensor.launch & \
roslaunch $HOME/catkin_ws/src/mobinha/selfdrive/manager/car.launch & \
roslaunch $HOME/catkin_ws/src/mobinha/selfdrive/manager/planner.launch

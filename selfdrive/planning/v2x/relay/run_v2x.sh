#!/bin/bash
sleep 2s

trap /home/inha/catkin_ws/src/niro/v2x/script/stop.sh SIGINT
python3 /home/inha/catkin_ws/src/niro/v2x/script/v2x_intersections.py

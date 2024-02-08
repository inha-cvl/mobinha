<p align="center">
  <img height="50" src="/docs/image/logo.png"/>
</p>


## What is mobinha ?

mobinha is a self driving system implementation project of INHA university.
mobinha performs the functions of Map-based self driving, Adaptive Cruise Control (ACC), Lane Keeping System (LKS)..

# Pull the code from main branch
## Catkin_Make "morai" folder
## Build dockerfile
## xhost +
## In docker container "pip3 install /workspace/src/docker/requirements.txt"
## apt install python3-pyqt5.qtmultimedia libqt5multimedia5-plugins
## apt install ros-noetic-jsk-recognition-msgs & apt install ros-noetic-jsk-rviz-plugins
## apt install ros-${ROS_DISTRO}-novatel-oem7-driver

Directory Structure
-----
    .
    ├── common              # Library like functionality we've developed here
    ├── docs                # Documentation
    ├── scripts             # system starts/stops script files
    ├── tools               # Unit tests, system tests, and a simulator
    └── selfdrive           # Code needed to drive the car
        ├── manager         # Daemon that starts/stops all other daemons as needed
        ├── car             # Car specific code to read states and control actuators
        ├── planning        
        ├── control 
        └── perception
        

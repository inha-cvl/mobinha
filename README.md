<p align="center">
  <img height="50" src="/docs/image/logo.png"/>
</p>

## What is mobinha ?

mobinha is a self driving system implementation project of INHA university.
mobinha performs the functions of Map-based self driving, Adaptive Cruise Control (ACC), Lane Keeping System (LKS)..

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
        

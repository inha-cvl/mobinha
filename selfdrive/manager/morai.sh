#!/bin/bash

python ../car/car.py &
python ../control/control.py &
python ../planning/planning.py &
python ../perception/perception.py &
python ../../../gen_ros/scripts/mobinha_planner.py &
python ../visualize/visualize.py 
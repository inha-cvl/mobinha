#!/bin/bash
python ../visualize/visualize.py 2> >(grep -v TF_REPEATED_DATA) &
sleep 3
python ../car/car.py &
python ../control/control.py &
python ../planning/planning.py &
python ../perception/perception.py 

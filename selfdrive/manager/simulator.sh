#!/bin/bash

python ../car/car.py &
python ../control/control.py &
python ../planning/planning.py &
python ../perception/perception.py &
#python ../perception/object_test.py &
python ../visualize/visualize.py 2> >(grep -v TF_REPEATED_DATA)
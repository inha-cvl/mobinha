#!/bin/bash

python ../car/car.py &
python ../control/control.py &
python ../planning/planning.py &
python ../perception/perception.py &
#python ../../tools/object_test.py &
python ../visualize/visualize.py 
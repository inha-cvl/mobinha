#!/bin/bash
python3 ../visualize/visualize.py 2> >(grep -v TF_REPEATED_DATA) &
sleep 3
python3 ../car/car.py &
python3 ../control/control.py &
python3 ../planning/planning.py &
python3 ../perception/perception.py 
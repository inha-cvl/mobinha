#!/usr/bin/python\
from car import control
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


CC = control.Control()

print(CC)

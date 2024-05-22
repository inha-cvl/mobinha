#!/usr/bin/python


from geometry_msgs.msg import Vector3

from libs.pid import PID, APID

from selfdrive.message.messaging import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


class Controller:
    def __init__(self):
        self.pid = PID()


    def calc_accel_brake_pressure(self, pid, pitch=0):
        if pid>0:
            accel_val = min(pid, 100)
            brake_val = 0
        else:
            accel_val = 0
            brake_val = max(pid, -100)
        
        # if pitch < -2.5:
        #     brake_val = 45
        # else:
        #     brake_val = 32
        
        return accel_val, brake_val
    
    def get_init_acuator(self):
        vector3 = Vector3()
        vector3.x = 0 #steer
        vector3.y = 0 #accel
        vector3.z = 32 #brakx
        return vector3 
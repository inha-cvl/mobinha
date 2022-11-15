#!/usr/bin/python
import numpy as np
import os
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker

dir_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(dir_path)

from config.config import Config
from libs.pid import PID
from libs.purepursuit import PurePursuit
from viz import *

class Controller:
    '''
    Unit
    velocity: m/s
    
    '''
    KPH_TO_MPS = 1 / 3.6
    MPS_TO_KPH = 3.6
    def __init__(self, stage=1):
        rospy.init_node('controller', anonymous=False)

        self.config = Config()
        self.pid = PID(self.config)
        #self.stage = rospy.get_param('stage', 1) # params file not exist is default : 1 
        self.purepursuit = PurePursuit(self.config, stage)

        self.target_v = 0.0
        self.get_odom = False
        self.final_path = None

        self.sub_odom = rospy.Subscriber('/odom', Float32MultiArray, self.odom_cb)
        if stage == 1:
            self.sub_final_path = rospy.Subscriber('/local_path', Marker, self.final_path_cb)
            #self.sub_final_path = rospy.Subscriber('/final_path', Marker, self.final_path_cb)

        elif stage == 2:   
            self.sub_final_path = rospy.Subscriber('/final_path', Marker, self.final_path_cb)

        self.sub_target_v = rospy.Subscriber('/target_v', Float32, self.target_v_cb)

        self.pub_wheel_angle = rospy.Publisher('/wheel_angle', Float32, queue_size=1)
        self.pub_accel_brake = rospy.Publisher('/accel_brake', Float32, queue_size=1)
        self.pub_lah = rospy.Publisher('/look_ahead', Marker, queue_size=1, latch=True)
        
    def odom_cb(self, msg):
        self.x, self.y, self.yaw, self.v, self.a = msg.data
        self.get_odom = True

    def final_path_cb(self, msg):
        self.final_path = [(pt.x, pt.y) for pt in msg.points]

    def target_v_cb(self, msg):
        self.target_v = msg.data*self.KPH_TO_MPS

    def run(self):
        rate = rospy.Rate(20) # 20hz

        while not rospy.is_shutdown():
            if self.get_odom:
                if self.final_path is None:
                    wheel_angle = 0.0
                else:
                    wheel_angle, lah_pt = self.purepursuit.run(self.x, self.y, self.yaw, self.v, self.final_path)
                    lah_viz = LookAheadViz(lah_pt)
                    self.pub_lah.publish(lah_viz)

                print('vvv', self.target_v)
                accel_brake = self.pid.run(self.target_v, self.v)
                self.pub_wheel_angle.publish(Float32(wheel_angle))
                self.pub_accel_brake.publish(Float32(accel_brake))

            rate.sleep()


if __name__ == "__main__":
    controller = Controller()
    controller.run()

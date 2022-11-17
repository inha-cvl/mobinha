# sudo ip link set can0 up type can bitrate 500000
import can
import cantools
import time
import tf
import math
import pymap3d
import os

import rospy

from sbg_driver.msg import SbgEkfNav, SbgEkfEuler
from std_msgs.msg import Int16, Float32, Float32MultiArray
from visualization_msgs.msg import Marker

from message.messaging import CS


class StateMaster:
    def __init__(self):
        self.bus = can.Th
        rospy.init_node('state_master', anonymous=False)
        self.sub_rtk_gps = rospy.Subscriber(
            '/sbg/ekf_nav', SbgEkfNav, self.rtk_gps_cb)
        self.sub_ins_imu = rospy.Subscriber(
            '/sbg/ekf_euler', SbgEkfEuler, self.ins_imu_cb)
        self.sub_ins_odom = rospy.Subscriber(
            '/car_v', Float32, self.ins_odom_cb)

            
        self.test = 0
        self.CS = CS

        self.receiver()
    
    def rtk_gps_cb(self, msg):
        self.x, self.y, _ = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, msg.altitude, self.base_lla[0], self.base_lla[1], self.base_lla[2])

    def ins_odom_cb(self, msg):
        self.v = msg.data

    def ins_imu_cb(self, msg):
        yaw = math.degrees(msg.angle.z)
        self.yaw = 90 - yaw if (yaw >= -90 and yaw <= 180) else -270 - yaw

    def receiver(self):
        while True:
            try:
                data = self.bus.recv()

            except KeyboardInterrupt:
                exit(0)

    def test_callback(self, msg):
        self.test = msg.data

    def update(self):
        car_state = self.CS._asdict()
        car_state["number"] = self.test
        self.CS = self.CS._make(car_state.values())

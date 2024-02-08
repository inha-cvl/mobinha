#!/usr/bin/python
import math
import tf

import rospy
from std_msgs.msg import Float32, Int8
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from sensor_msgs.msg import Imu

from novatel_oem7_msgs.msg import INSPVA


class MoraiTransceiver:
    def __init__(self):
        self.gps_ok = False
        self.imu_ok = False

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.pub_novatel = rospy.Publisher('/novatel/oem7/inspva', INSPVA, queue_size=1)
        self.pub_velocity = rospy.Publisher('/mobinha/car/velocity', Float32, queue_size=1)
        self.pub_mode = rospy.Publisher('/mobinha/car/mode', Int8, queue_size=1)
        #From MORAI
        rospy.Subscriber("/gps", GPSMessage, self.gps_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)

    def gps_cb(self, msg):
        self.gps_ok = True
        self.ll_position = (msg.latitude, msg.longitude, msg.altitude)

    def imu_cb(self, msg):
        self.imu_ok = True
        quaternion = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(
            quaternion)

    def can_cmd(self, canCmd):
        mode = 0
        if canCmd.enable:
            mode = 1
        self.pub_mode.publish(Int8(mode))

    def ego_topic_cb(self, msg):
        self.imu_ok = True
        self.heading = msg.heading-30
        self.velocity = msg.velocity.x

    def run(self, CM=None):
        if self.gps_ok and self.imu_ok:
            self.can_cmd(CM.CC.canCmd)
            msg = INSPVA()
            msg.latitude = self.ll_position[0]
            msg.longitude = self.ll_position[1]
            msg.height = self.ll_position[2]
            msg.roll = self.roll
            msg.pitch = self.pitch
            msg.azimuth = -(math.degrees(self.yaw)+270)
            self.pub_novatel.publish(msg)

            self.pub_velocity.publish(Float32(self.velocity))

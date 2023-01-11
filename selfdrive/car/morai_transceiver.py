#!/usr/bin/python
import math
import tf

import rospy
from std_msgs.msg import Float32
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from sensor_msgs.msg import Imu

from sbg_driver.msg import SbgEkfNav, SbgEkfEuler


class MoraiTransceiver:
    def __init__(self):
        self.gps_ok = False
        self.imu_ok = False

        self.roll = 0.0
        self.pitch = 0.0

        self.pub_rtk_gps = rospy.Publisher(
            '/sbg/ekf_nav', SbgEkfNav, queue_size=1)
        self.pub_ins_imu = rospy.Publisher(
            '/sbg/ekf_euler', SbgEkfEuler, queue_size=1)
        self.pub_ins_odom = rospy.Publisher('/car_v', Float32, queue_size=1)

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
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            quaternion)
        yaw = math.degrees(yaw)
        self.roll = roll
        self.pitch = pitch
        # self.heading = 90 - \
        #     yaw if (yaw >= -90 and yaw <= 180) else -270 - yaw
        # self.velocity = msg.linear_acceleration.x

    def ego_topic_cb(self, msg):
        self.imu_ok = True
        self.heading = 90 - \
            msg.heading if (msg.heading >= -90 and msg.heading <=
                            180) else -270 - msg.heading
        self.velocity = msg.velocity.x

    def run(self):
        if self.gps_ok and self.imu_ok:

            msg = SbgEkfNav()
            msg.latitude = self.ll_position[0]
            msg.longitude = self.ll_position[1]
            msg.altitude = self.ll_position[2]
            self.pub_rtk_gps.publish(msg)

            msg = SbgEkfEuler()
            yaw = self.heading
            yaw = math.radians(yaw)
            msg.angle.x = self.roll
            msg.angle.y = self.pitch
            msg.angle.z = yaw

            self.pub_ins_imu.publish(msg)
            self.pub_ins_odom.publish(Float32(self.velocity))

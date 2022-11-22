#!/usr/bin/python
import tf
import math
import pymap3d
import signal
import sys

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from sbg_driver.msg import SbgEkfNav, SbgEkfEuler


class Vehicle:
    def __init__(self, x, y, yaw, v, L):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L

    def set(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def next_state(self, dt, wheel_angle, accel_brake):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v * dt * math.tan(wheel_angle) / self.L
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.v = max(0, self.v + accel_brake * dt)
        return self.x, self.y, self.yaw, self.v


class SimulatorTransceiver:
    def __init__(self, CP):

        self.base_lla = [CP.mapParam.baseLatitude,
                         CP.mapParam.baseLongitude, CP.mapParam.baseAltitude]

        self.wheel_angle = 0.0
        self.accel_brake = 0.0

        self.ego = Vehicle(0.0, 0.0, math.radians(-60), 0.0, 2.65)

        self.pub_rtk_gps = rospy.Publisher(
            '/sbg/ekf_nav', SbgEkfNav, queue_size=1)
        self.pub_ins_imu = rospy.Publisher(
            '/sbg/ekf_euler', SbgEkfEuler, queue_size=1)
        self.pub_ins_odom = rospy.Publisher('/car_v', Float32, queue_size=1)

        self.sub_initpose = rospy.Subscriber(
            '/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        self.sub_wheel_angle = rospy.Subscriber(
            '/wheel_angle', Float32, self.wheel_angle_cb)
        self.sub_accel_brake = rospy.Subscriber(
            '/accel_brake', Float32, self.accel_brake_cb)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,
                      orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        self.ego.set(x, y, yaw)

    def wheel_angle_cb(self, msg):
        self.wheel_angle = math.radians(msg.data)

    def accel_brake_cb(self, msg):
        self.accel_brake = msg.data

    def run(self):
        dt = 0.1
        x, y, yaw, v = self.ego.next_state(
            dt, self.wheel_angle, self.accel_brake)

        msg = SbgEkfNav()
        lat, lon, alt = pymap3d.enu2geodetic(
            x, y, self.base_lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.pub_rtk_gps.publish(msg)

        msg = SbgEkfEuler()
        yaw = math.degrees(yaw)
        yaw = -270 - yaw if (yaw >= -180 and yaw <= -90) else -yaw + 90
        yaw = math.radians(yaw)
        msg.angle.z = yaw
        self.pub_ins_imu.publish(msg)

        self.pub_ins_odom.publish(Float32(v))

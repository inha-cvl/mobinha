#!/usr/bin/python
import tf
import math
import pymap3d

import rospy
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import PoseWithCovarianceStamped
from novatel_oem7_msgs.msg import INSPVA


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
        self.gear = 0

        # posco 957.413, -851.312, terminal -1601.567, 2375.599
        self.ego = Vehicle(957.413, -851.312, math.radians(180), 0.0, 2.65)
        self.roll = 0.0
        self.pitch = 0.0

        self.pub_novatel = rospy.Publisher(
            '/novatel/oem7/inspva', INSPVA, queue_size=1)
        self.pub_velocity = rospy.Publisher(
            '/mobinha/car/velocity', Float32, queue_size=1)
        self.pub_gear = rospy.Publisher(
            '/mobinha/car/gear', Int8, queue_size=1)

        rospy.Subscriber(
            '/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        rospy.Subscriber(
            '/mobinha/control/wheel_angle', Float32, self.wheel_angle_cb)
        rospy.Subscriber(
            '/mobinha/control/accel_brake', Float32, self.accel_brake_cb)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,
                      orientation.z, orientation.w)
        self.roll, self.pitch, yaw = tf.transformations.euler_from_quaternion(
            quaternion)
        self.ego.set(x, y, yaw)

    def wheel_angle_cb(self, msg):
        self.wheel_angle = math.radians(msg.data)

    def accel_brake_cb(self, msg):
        self.accel_brake = msg.data

    def run(self):
        self.pub_gear.publish(self.gear)

        dt = 0.1
        x, y, yaw, v = self.ego.next_state(
            dt, self.wheel_angle, self.accel_brake)

        if v > 0:
            self.gear = 3
        else:
            self.gear = 0

        inspva = INSPVA()
        lat, lon, alt = pymap3d.enu2geodetic(
            x, y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        inspva.latitude = lat
        inspva.longitude = lon
        inspva.height = alt
        inspva.roll = self.roll
        inspva.pitch = self.pitch
        inspva.azimuth = math.degrees(yaw)

        self.pub_novatel.publish(inspva)
        self.pub_velocity.publish(Float32(v))

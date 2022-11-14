#!/usr/bin/python
import tf
import math
import pymap3d

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped


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


class EgoSimulate:
    def __init__(self):
        rospy.init_node('ego_simulate', anonymous=False)

        self.base_lla = rospy.get_param("base_lla")

        self.wheel_angle = 0.0
        self.accel_brake = 0.0

        self.ego = Vehicle(0.0, 0.0, math.radians(0.0), 0.0, 2.65)

        self.pub_rtk_gps = rospy.Publisher('/fix', NavSatFix, queue_size=1)
        self.pub_ins_odom = rospy.Publisher('/vectornav/Odom', Odometry, queue_size=1)
        self.pub_ins_imu = rospy.Publisher('/vectornav/IMU', Imu, queue_size=1)

        self.sub_initpose = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        self.sub_wheel_angle = rospy.Subscriber('/wheel_angle', Float32, self.wheel_angle_cb)
        self.sub_accel_brake = rospy.Subscriber('/accel_brake', Float32, self.accel_brake_cb)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        self.ego.set(x, y, yaw)

    def wheel_angle_cb(self, msg):
        self.wheel_angle = math.radians(msg.data)

    def accel_brake_cb(self, msg):
        self.accel_brake = msg.data

    def run(self):
        dt = 0.1
        rate = rospy.Rate(1 / dt) # 10hz

        while not rospy.is_shutdown():
            x, y, yaw, v = self.ego.next_state(dt, self.wheel_angle, self.accel_brake)

            msg = Odometry()
            msg.twist.twist.linear.x = v
            self.pub_ins_odom.publish(msg)

            msg = Imu()
            yaw = math.degrees(yaw)
            yaw = -270 - yaw if (yaw >= -180 and yaw <= -90) else -yaw + 90
            yaw = math.radians(yaw)
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            msg.orientation.x = quaternion[0]
            msg.orientation.y = quaternion[1]
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]
            self.pub_ins_imu.publish(msg)
            
            msg = NavSatFix()
            msg.status.status = 2
            lat, lon, alt = pymap3d.enu2geodetic(x, y, self.base_lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            self.pub_rtk_gps.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    ego_simulate = EgoSimulate()
    ego_simulate.run()
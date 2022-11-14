#!/usr/bin/python
import tf
import math
import pymap3d
import os

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

dir_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(dir_path)

from viz import *

class Localizer:
    def __init__(self):
        rospy.init_node('localizer', anonymous=False)

        self.base_lla = rospy.get_param("base_lla")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.a = 0.0

        self.ego_car = EgoCarViz()

        self.br = tf.TransformBroadcaster()

        self.pub_odom = rospy.Publisher('/odom', Float32MultiArray, queue_size=1, latch=True)
        self.pub_ego_car = rospy.Publisher('/ego_car', Marker, queue_size=1)

        self.sub_rtk_gps = rospy.Subscriber('/fix', NavSatFix, self.rtk_gps_cb)
        self.sub_ins_odom = rospy.Subscriber('/vectornav/Odom', Odometry, self.ins_odom_cb)
        self.sub_ins_imu = rospy.Subscriber('/vectornav/IMU', Imu, self.ins_imu_cb)

    def rtk_gps_cb(self, msg):
        if msg.status.status != -1:
            self.x, self.y, _ = pymap3d.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, self.base_lla[0], self.base_lla[1], self.base_lla[2])

    def ins_odom_cb(self, msg):
        self.v = msg.twist.twist.linear.x

    def ins_imu_cb(self, msg):
        orientation = msg.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        yaw = math.degrees(yaw)
        self.yaw = 90 - yaw if (yaw >= -90 and yaw <= 180) else -270 - yaw
        self.a = msg.linear_acceleration.x

    def run(self):
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            odom_msg = Float32MultiArray()
            odom_msg.data = [self.x, self.y, self.yaw, self.v, self.a]
            self.pub_odom.publish(odom_msg)

            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, math.radians(self.yaw))
            self.br.sendTransform(
                                (self.x, self.y, 0.0),
                                (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                                rospy.Time.now(),
                                'ego_car',
                                'world'
                             )
            self.pub_ego_car.publish(self.ego_car)

            rate.sleep()


if __name__ == "__main__":
    localizer = Localizer()
    localizer.run()
import math
import pymap3d

import rospy
from sbg_driver.msg import SbgEkfNav, SbgEkfEuler
from std_msgs.msg import Float32

from message.messaging import CS


class StateMaster:
    def __init__(self, CP):
        rospy.init_node('state_master', anonymous=False)

        self.sub_rtk_gps = rospy.Subscriber(
            '/sbg/ekf_nav', SbgEkfNav, self.rtk_gps_cb)
        self.sub_ins_imu = rospy.Subscriber(
            '/sbg/ekf_euler', SbgEkfEuler, self.ins_imu_cb)
        self.sub_ins_odom = rospy.Subscriber(
            '/car_v', Float32, self.ins_odom_cb)

        self.CS = CS
        self.base_lla = [CP.mapParam.baseLatitude,
                         CP.mapParam.baseLongitude, CP.mapParam.baseAltitude]

        self.v = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0

    def rtk_gps_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        self.x, self.y, _ = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, msg.altitude, self.base_lla[0], self.base_lla[1], self.base_lla[2])

    def ins_odom_cb(self, msg):
        self.v = msg.data

    def ins_imu_cb(self, msg):
        yaw = math.degrees(msg.angle.z)
        self.yaw = 90 - yaw if (yaw >= -90 and yaw <= 180) else -270 - yaw

    def update(self):
        car_state = self.CS._asdict()

        car_state["vEgo"] = self.v
        car_state_position = car_state["position"]._sedict()
        car_state_position["x"] = self.x
        car_state_position["y"] = self.y
        car_state_position["latitude"] = self.latitude
        car_state_position["longitude"] = self.longitude
        car_state_position["altitude"] = self.altitude
        car_state["position"] = self.CS.position._make(
            car_state_position.values())
        car_state["yawRate"] = self.yaw

        self.CS = self.CS._make(car_state.values())

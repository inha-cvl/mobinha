import math
import pymap3d

import rospy
from novatel_oem7_msgs.msg import INSPVA
import math
from std_msgs.msg import Int8, Float32

from selfdrive.message.car_message import car_state

CS = car_state.CarState()


class StateMaster:
    def __init__(self, CP):
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)
        rospy.Subscriber('/mobinha/car/velocity', Float32, self.velocity_cb)
        rospy.Subscriber('/mobinha/car/gear', Int8, self.gear_cb)
        rospy.Subscriber('/mobinha/planning/blinker', Int8, self.blinker_cb)

        self.CS = CS
        self.base_lla = [CP.mapParam.baseLatitude,
                         CP.mapParam.baseLongitude, CP.mapParam.baseAltitude]

        self.v = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.gear = 0
        self.blinker = 0

    def novatel_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.height
        self.x, self.y, self.z = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, msg.height, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.roll = math.degrees(msg.roll)
        self.pitch = math.degrees(msg.pitch)
        self.yaw = msg.azimuth
        #90 - msg.azimuth if (msg.azimuth >= -90 and msg.azimuth <=180) else -270 - msg.azimuth

    def velocity_cb(self, msg):
        self.v = msg.data

    def gear_cb(self, msg):
        self.gear = msg.data

    def blinker_cb(self, msg):
        self.blinker = msg.data  # 0:stay 1:left 2:right

    def update(self):
        car_state = self.CS._asdict()

        car_state["vEgo"] = self.v
        car_state_position = car_state["position"]._asdict()
        car_state_position["x"] = self.x
        car_state_position["y"] = self.y
        car_state_position["z"] = self.z
        car_state_position["latitude"] = self.latitude
        car_state_position["longitude"] = self.longitude
        car_state_position["altitude"] = self.altitude
        car_state["position"] = self.CS.position._make(
            car_state_position.values())
        car_state["yawRate"] = self.yaw
        car_state["pitchRate"] = self.pitch
        car_state["rollRate"] = self.roll
        car_state["gearShifter"] = self.gear
        car_state_button_event = car_state["buttonEvent"]._asdict()
        car_state_button_event["leftBlinker"] = 1 if self.blinker == 1 else 0
        car_state_button_event["rightBlinker"] = 1 if self.blinker == 2 else 0
        car_state["buttonEvent"] = self.CS.buttonEvent._make(
            car_state_button_event.values())
        self.CS = self.CS._make(car_state.values())

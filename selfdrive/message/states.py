import math
import pymap3d
import tf
import rospy
from novatel_oem7_msgs.msg import INSPVA
import math
from std_msgs.msg import Int8, Float32, Int8MultiArray, Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, NavSatFix
import time

from selfdrive.message.car_message import car_state

CS = car_state.CarState()

NOVATEL_OK = True


class StateMaster:
    def __init__(self, CP):

        self.CS = CS
        self.base_lla = [CP.mapParam.baseLatitude,CP.mapParam.baseLongitude, CP.mapParam.baseAltitude]

        self.v = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.x = 0
        self.y = 0
        self.z = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.gear = 0
        self.mode = 0
        self.blinker = 0
        self.ego_actuators = {"steer":0.0, "accel":0.0, "brake":0.0}
        self.gateway_time = 0.0

        if NOVATEL_OK:
            rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)
        else:
            rospy.Subscriber('/gps/imu', Imu, self.imu_cb)
            rospy.Subscriber('/gps/fix', NavSatFix, self.gps_cb)

        rospy.Subscriber('/mobinha/car/velocity', Float32, self.velocity_cb)
        rospy.Subscriber('/mobinha/car/gear', Int8, self.gear_cb)
        rospy.Subscriber('/mobinha/car/mode', Int8, self.mode_cb)
        rospy.Subscriber('/mobinha/planning/blinker', Int8, self.blinker_cb)
        rospy.Subscriber('/mobinha/car/ego_actuators',Vector3, self.ego_actuators_cb)
        # rospy.Subscriber('/mobinha/car/gateway', Int8MultiArray, self.gateway_cb)
        rospy.Subscriber('/mobinha/car/gateway_time', Float64, self.gateway_time_cb)
        self.gateway_check = rospy.Publisher('/mobinha/car/gateway_state', Int8, queue_size=1)
        
    #     self.tick = {1: 0, 0.5: 0}

    # def timer(self, sec):
    #     if time.time() - self.tick[sec] > sec:
    #         self.tick[sec] = time.time()
    #         return True
    #     else:
    #         return False
        
    def imu_cb(self, msg):
        orientation = msg.orientation
        quaternion = (orientation.x, orientation.y,
                      orientation.z, orientation.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = math.degrees(roll)
        self.pitch = math.degrees(pitch)
        self.yaw = math.degrees(yaw)
        # self.yaw = 90 + yaw if (yaw >= -90 and yaw <=180) else -270 + yaw

    def gps_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        self.x, self.y, self.z = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, 0, self.base_lla[0], self.base_lla[1], 0)

    def novatel_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.height
        self.x, self.y, self.z = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, 0, self.base_lla[0], self.base_lla[1], 0)
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = 90 - msg.azimuth + 360 if (-270 <= 90 - msg.azimuth <= -180) else 90 - msg.azimuth
        self.yaw = self.yaw - 0.3
       
    def velocity_cb(self, msg):
        self.v = msg.data

    def gear_cb(self, msg):
        self.gear = msg.data

    def mode_cb(self, msg):
        self.mode = msg.data

    def blinker_cb(self, msg):
        self.blinker = msg.data  # 0:stay 1:left 2:right

    def ego_actuators_cb(self, msg):
        self.ego_actuators["steer"] = msg.x
        self.ego_actuators["accel"] = msg.y
        self.ego_actuators["brake"] = msg.z
    
    def gateway_time_cb(self, msg):
        self.gateway_time = msg.data

    def checker(self):
        gateway_state = Int8()
        if self.mode == 1: # normal
            if time.time() - self.gateway_time < 1:
                gateway_state.data = 1
            else: # abnormals
                gateway_state.data = 0
        else:
            gateway_state.data = 1
                
        self.gateway_check.publish(gateway_state)

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
        car_state["actuators"] = self.CS.actuators._make(
            self.ego_actuators.values())
        car_state["gearShifter"] = self.gear
        car_state["cruiseState"] = self.mode
        car_state_button_event = car_state["buttonEvent"]._asdict()
        car_state_button_event["leftBlinker"] = 1 if self.blinker == 1 else 0
        car_state_button_event["rightBlinker"] = 1 if self.blinker == 2 else 0
        car_state["buttonEvent"] = self.CS.buttonEvent._make(
            car_state_button_event.values())
        self.CS = self.CS._make(car_state.values())
        self.checker()

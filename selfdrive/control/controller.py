#!/usr/bin/python
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Vector3
from selfdrive.visualize.rviz_utils import *
from selfdrive.control.libs.purepursuit import PurePursuit
from selfdrive.control.libs.pid import PID
import rospy

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


class Controller:

    def __init__(self, CP):
        self.pid = PID(CP.longitudinalTuning)
        self.purepursuit = PurePursuit(CP)
        self.steer_ratio = CP.steerRatio
        self.target_v = 0.0
        self.local_path = None
        self.l_idx = 0
        self.prev_steer = 0.0

        rospy.Subscriber('/mobinha/planning/local_path', Marker, self.local_path_cb)
        rospy.Subscriber('/mobinha/planning/target_v', Float32, self.target_v_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        self.pub_target_actuators = rospy.Publisher('/mobinha/control/target_actuators', Vector3, queue_size=1)
        self.pub_lah = rospy.Publisher('/mobinha/control/look_ahead', Marker, queue_size=1, latch=True)

    def limit_steer_change(self, steer):
        #TODO:limit logic error need modified 
        # steer_diff = steer - self.prev_steer
        # if abs(steer_diff) > 10:
        #     steer = self.prev_steer + (10 if steer_diff > 0 else -10)
        # else:
        #     self.prev_steer = steer
        # self.prev_steer = steer
        return steer
    
    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def target_v_cb(self, msg):
        self.target_v = msg.data

    def lane_information_cb(self, msg):
        self.l_idx = msg.orientation.y

    def calc_accel_brake_pressure(self, pid, cur_v):
        # th_a = 4 # 0~20 * gain -> 0~100 accel
        # th_b = 13 # 0~20 * gain -> 0~100 brake
        # val_data = max(-th_b, min(th_a, pid))
        # gain = 5
        # if val_data > 0.:
        #     accel_val = val_data*gain
        #     brake_val = 0.0
        # elif val_data <= 0.:
        #     accel_val = 0.0
        #     brake_val = (-val_data/th_b)**1.1*th_b*gain if (self.target_v > 0 and cur_v >= 3*KPH_TO_MPS) else 35
        th_a = 4 # 0~20 * gain -> 0~100 accel
        th_b = 13 # 0~20 * gain -> 0~100 brake
        gain = 5
        val_data = max(-th_b*gain, min(th_a*gain, pid))
        if val_data > 0.:
            accel_val = val_data
            brake_val = 0.0
        elif val_data <= 0.:
            accel_val = 0.0
            brake_val = -val_data if (self.target_v > 0 and cur_v >= 3*KPH_TO_MPS) else 35
        
        return accel_val, brake_val
    
    def get_init_acuator(self):
        vector3 = Vector3()
        vector3.x = 0 #steer
        vector3.y = 0 #accel
        vector3.z = 35 #brakx
        return vector3 
    
    def run(self, sm):
        CS = sm.CS
        vector3 = self.get_init_acuator()
        if self.local_path != None:
            wheel_angle, lah_pt = self.purepursuit.run(
                CS.vEgo, self.local_path[int(self.l_idx):], (CS.position.x, CS.position.y), CS.yawRate)
            steer = wheel_angle*self.steer_ratio
            # print("origin steer:",steer)
            steer = self.limit_steer_change(steer)
            # print("limit steer:",steer)
            lah_viz = LookAheadViz(lah_pt)
            self.pub_lah.publish(lah_viz)
            pid = self.pid.run(self.target_v, CS.vEgo) #-100~100
            accel, brake = self.calc_accel_brake_pressure(pid, CS.vEgo)
            
            vector3.x = steer
            vector3.y = accel
            vector3.z = brake

        if CS.cruiseState != 1:
            vector3.x = CS.actuators.steer
            vector3.y = CS.actuators.accel
            vector3.z = CS.actuators.brake

        self.pub_target_actuators.publish(vector3)

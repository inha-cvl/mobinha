#!/usr/bin/python
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Pose, Vector3
from selfdrive.visualize.rviz_utils import *
from selfdrive.control.libs.purepursuit import PurePursuit
from selfdrive.control.libs.stanley import StanleyController
from selfdrive.control.libs.pid import PID
import rospy
from selfdrive.planning.libs.map import LaneletMap, TileMap
from selfdrive.message.messaging import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
HZ=20

class Controller:
    def __init__(self, CP):
        self.lmap = LaneletMap(CP.mapParam.path)
        self.pid = PID(CP.longitudinalTuning)
        self.purepursuit = PurePursuit(CP)
        self.stanley = StanleyController(CP)
        self.steer_ratio = CP.steerRatio
        self.target_v = 0.0
        self.local_path = None
        self.l_idx = 0
        self.prev_steer = 0.0
        self.local_path_theta = None
        self.local_path_radius = None
        self.local_path_k = None        
        self.max_steer_change_rate = 8/HZ*self.steer_ratio # SONGDO 8 KCITY10
        self.car = rospy.get_param('car_name', 'None')
        self.cte = 0
        rospy.Subscriber('/mobinha/planning/local_path', Marker, self.local_path_cb)
        rospy.Subscriber('/mobinha/planning/target_v', Float32, self.target_v_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        # rospy.Subscriber('mobinha/planning/local_path_theta', Float32MultiArray, self.local_path_theta_cb)
        # rospy.Subscriber('mobinha/planning/local_path_radius', Float32MultiArray, self.local_path_radius_cb)
        # rospy.Subscriber('mobinha/planning/local_path_k', Float32MultiArray, self.local_path_k_cb)
        rospy.Subscriber('/mobinha/planning/goal_information',Pose, self.goal_information_cb)
        self.pub_target_actuators = rospy.Publisher('/mobinha/control/target_actuators', Vector3, queue_size=1)
        self.pub_lah = rospy.Publisher('/mobinha/control/look_ahead', Marker, queue_size=1, latch=True)

        self.car = rospy.get_param('car_name', 'None')

    def limit_steer_change(self, current_steer):
        steer_change = current_steer - self.prev_steer
        steer_change = np.clip(steer_change, -self.max_steer_change_rate, self.max_steer_change_rate)
        limited_steer = self.prev_steer + steer_change
        self.prev_steer = limited_steer
        return limited_steer

    def goal_information_cb(self, msg):
        self.cte = msg.orientation.y
        
    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]
    
    def local_path_theta_cb(self, msg):
        self.local_path_theta = msg.data

    def local_path_radius_cb(self, msg):
        self.local_path_radius = msg.data
    
    def local_path_k_cb(self, msg):
        self.local_path_k = msg.data

    def target_v_cb(self, msg):
        self.target_v = msg.data

    def lane_information_cb(self, msg):
        self.l_idx = msg.orientation.y

    def calc_accel_brake_pressure(self, pid, cur_v, pitch):
        th_a = 4 # 0~20 * gain -> 0~100 accel origin a,b = 4, 13
        th_b = 13 # 0~20 * gain -> 0~100 brake
        gain = 5
        val_data = max(-th_b, min(th_a, pid))
        if val_data > 0.:
            accel_val = val_data*gain
            brake_val = 0.0
        elif val_data <= 0.:
            accel_val = 0.0
            if (self.target_v > 0 and cur_v >= 1.5*KPH_TO_MPS):
                brake_val = -val_data*gain
            elif pitch < -2.5:
                brake_val = 42
            else:
                brake_val = 32
        
        return accel_val, brake_val
    
    def morai_accel_brake(self, pid, cur_v, pitch):
        if pid > 0.:
            accel_val = pid
            brake_val = 0.0
        elif pid <= 0.:
            accel_val = 0.0
            # if (self.target_v > 0 and cur_v >= 1.5*KPH_TO_MPS):
            brake_val = max(-pid - 5, 0)
            # else:
                # brake_val = 5
        return accel_val, brake_val
    
    def get_init_acuator(self):
        vector3 = Vector3()
        vector3.x = 0 #steer
        vector3.y = 0 #accel
        vector3.z = 32 #brakx
        return vector3
    
    def run(self, sm):
        CS = sm.CS
        vector3 = self.get_init_acuator()
        if self.local_path != None:
            flagHybrid = True
            flagstanley = False
            if flagHybrid:
                wheel_angle, lah_pt = self.purepursuit.run(CS.vEgo, self.local_path[int(self.l_idx):], (CS.position.x, CS.position.y), CS.yawRate, self.cte, flagHybrid)
            elif flagstanley:
                wheel_angle = self.stanley.run(CS.vEgo, self.local_path[int(self.l_idx):], (CS.position.x, CS.position.y), CS.yawRate)
            else:
                wheel_angle, lah_pt = self.purepursuit.run(CS.vEgo, self.local_path[int(self.l_idx):], (CS.position.x, CS.position.y), CS.yawRate, self.cte, flagHybrid)
            
            steer = wheel_angle*self.steer_ratio
            steer = self.limit_steer_change(steer)

            lah_viz = LookAheadViz(lah_pt)
            self.pub_lah.publish(lah_viz)

            pid = self.pid.run(self.target_v, CS.vEgo) #-100~100
            accel, brake = self.calc_accel_brake_pressure(pid, CS.vEgo, CS.pitchRate)
            if self.car =="MORAI":
                accel, brake = self.morai_accel_brake(pid, CS.vEgo, CS.pitchRate)
                print("morai accel:",round(accel),"brake:",round(brake))
            
            vector3.x = steer
            vector3.y = accel
            vector3.z = brake

        if CS.cruiseState != 1:
            vector3.x = CS.actuators.steer
            vector3.y = CS.actuators.accel
            vector3.z = CS.actuators.brake

        self.pub_target_actuators.publish(vector3)
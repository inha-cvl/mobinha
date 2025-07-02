#!/usr/bin/python
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Pose, Vector3
from selfdrive.visualize.rviz_utils import *
from selfdrive.control.libs.purepursuit import PurePursuit
from selfdrive.control.libs.pid import PID
import rospy
from selfdrive.planning.libs.map import LaneletMap, TileMap
from selfdrive.message.messaging import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


class Controller:

    def __init__(self, CP):
        self.lmap = LaneletMap(CP.mapParam.path)
        self.pid = PID(CP.longitudinalTuning)
        self.purepursuit = PurePursuit(CP)
        self.steer_ratio = CP.steerRatio
        self.target_v = 0.0
        self.local_path = None
        self.l_idx = 0
        self.prev_steer = 0.0
        self.max_steer_delta = 40/20*self.steer_ratio
        self.cte = 0
        self.actuator_steer = 0
        self.actuator_accel = 0
        self.actuator_brake = 0

        rospy.Subscriber('/mobinha/planning/local_path', Marker, self.local_path_cb)
        rospy.Subscriber('/mobinha/planning/target_v', Float32, self.target_v_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        # rospy.Subscriber('/mobinha/planning/goal_information',Pose, self.goal_information_cb)
        rospy.Subscriber('/mobinha/planning/cte', Float32, self.cte_cb)
        rospy.Subscriber('/mobinha/car/ego_actuators', Vector3, self.actuator_cb)

        self.pub_target_actuators = rospy.Publisher('/mobinha/control/target_actuators', Vector3, queue_size=1)
        self.pub_lah =              rospy.Publisher('/mobinha/control/look_ahead', Marker, queue_size=1, latch=True)
        # rospy.Subscriber('mobinha/planning/local_path_theta', Float32MultiArray, self.local_path_theta_cb)
        # rospy.Subscriber('mobinha/planning/local_path_radius', Float32MultiArray, self.local_path_radius_cb)
        # rospy.Subscriber('mobinha/planning/local_path_k', Float32MultiArray, self.local_path_k_cb)

        self.local_path_theta = None
        self.local_path_radius = None
        self.local_path_k = None

        self.car = rospy.get_param('car_name', 'None')

    def limit_steer_change(self, current_steer):
        steer_change = current_steer - self.actuator_steer
        steer_change = np.clip(steer_change, -self.max_steer_delta, self.max_steer_delta)
        limited_steer = self.actuator_steer + steer_change
        
        return limited_steer
    
    def limit_accel_change(self, accel_cmd):
        if accel_cmd >= 0.0:
            delta = accel_cmd - self.actuator_accel
            delta = np.clip(delta, -5, 5)
            limited_accel = self.actuator_accel + delta
            self.actuator_accel = limited_accel
            self.actuator_brake = 0.0   
            return limited_accel, 0.0   

        else:  
            brake_cmd = -accel_cmd   
            delta = brake_cmd - self.actuator_brake
            delta = np.clip(delta, -5, 5)
            limited_brake = self.actuator_brake + delta
            self.actuator_brake = limited_brake
            self.actuator_accel = 0.0    
            return 0.0, limited_brake
    
    def actuator_cb(self, msg):
        self.actuator_steer = msg.x
        self.actuator_accel = msg.y
        self.actuator_brake = msg.z

    def cte_cb(self, msg):
        self.cte = msg.data
    
    def local_path_theta_cb(self, msg):
        self.local_path_theta = msg.data

    def local_path_radius_cb(self, msg):
        self.local_path_radius = msg.data
    
    def local_path_k_cb(self, msg):
        self.local_path_k = msg.data
        
    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def target_v_cb(self, msg):
        self.target_v = msg.data

    def lane_information_cb(self, msg):
        self.l_idx = msg.orientation.y

    def calc_accel_brake_pressure(self, pid, cur_v, pitch):
        th_a = 4 # 0~20 * gain -> 0~100 accel
        th_b = 13 # 0~20 * gain -> 0~100 brake
        gain = 5
        val_data = max(-th_b, min(th_a, pid))
        brake_val = 0
        if val_data > 0.:
            accel_val = val_data*gain
            brake_val = 0.0
        elif val_data <= 0.:
            accel_val = 0.0
            if (self.target_v == -1 and cur_v <= 10*KPH_TO_MPS):
                brake_val = 40
            elif (self.target_v > 0 and cur_v >= 1.5*KPH_TO_MPS):
                brake_val = -val_data*gain

        
            # elif pitch < -2.5:
            #     brake_val = 45
            # else:
            #     brake_val = 32
        
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
            wheel_angle, lah_pt = self.purepursuit.run(
                CS.vEgo, self.local_path[int(self.l_idx):], (CS.position.x, CS.position.y), CS.yawRate, self.cte)
            
            lah_viz = LookAheadViz(lah_pt)
            self.pub_lah.publish(lah_viz)

            steer = wheel_angle*self.steer_ratio
            steer = self.limit_steer_change(steer)

            pid = self.pid.run(self.target_v, CS.vEgo) #-100~100
            accel, brake = self.calc_accel_brake_pressure(pid, CS.vEgo, CS.pitchRate)
            # accel, brake = self.limit_accel_change(pid)
            print(f"accel: {accel:.2f}, brake: {brake:.2f}")
            
            vector3.x = steer
            vector3.y = accel
            vector3.z = brake

        if CS.cruiseState != 1:
            vector3.x = CS.actuators.steer
            vector3.y = CS.actuators.accel
            vector3.z = CS.actuators.brake

        self.pub_target_actuators.publish(vector3)
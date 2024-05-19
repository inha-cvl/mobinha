#!/usr/bin/python
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Pose, Vector3
from selfdrive.visualize.rviz_utils import *
from selfdrive.control.libs.purepursuit import PurePursuit
from selfdrive.control.libs.pid import PID, APID
import rospy
from selfdrive.planning.libs.map import LaneletMap, TileMap
from selfdrive.message.messaging import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


class Controller:

    def __init__(self, CP):
        self.lmap = LaneletMap(CP.mapParam.path)
        self.pid = PID(CP.longitudinalTuning)
        self.apid = APID()
        self.purepursuit = PurePursuit(CP)
        self.steer_ratio = CP.steerRatio
        self.target_v = 0.0
        self.local_path = None
        self.l_idx = 0
        self.prev_steer = 0.0
        self.max_steer_change_rate = self.steer_ratio
        # self.max_steer_change_rate = 8/20*self.steer_ratio
        self.cte = 0

        rospy.Subscriber('/mobinha/planning/local_path', Marker, self.local_path_cb)
        rospy.Subscriber('/mobinha/planning/target_v', Float32, self.target_v_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        rospy.Subscriber('/mobinha/planning/goal_information',Pose, self.goal_information_cb)
        self.pub_target_actuators = rospy.Publisher('/mobinha/control/target_actuators', Vector3, queue_size=1)
        self.pub_lah = rospy.Publisher('/mobinha/control/look_ahead', Marker, queue_size=1, latch=True)
        # rospy.Subscriber('mobinha/planning/local_path_theta', Float32MultiArray, self.local_path_theta_cb)
        # rospy.Subscriber('mobinha/planning/local_path_radius', Float32MultiArray, self.local_path_radius_cb)
        # rospy.Subscriber('mobinha/planning/local_path_k', Float32MultiArray, self.local_path_k_cb)

        self.local_path_theta = None
        self.local_path_radius = None
        self.local_path_k = None

        self.car = rospy.get_param('car_name', 'None')

    def limit_steer_change(self, current_steer):
        steer_change = current_steer - self.prev_steer
        # steer_change = np.clip(steer_change, -self.max_steer_change_rate, self.max_steer_change_rate)
        limited_steer = self.prev_steer + steer_change
        self.prev_steer = limited_steer
        return limited_steer

    def goal_information_cb(self, msg):
        self.cte = msg.orientation.y
    
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
            
            threshold = 449
            steer = int(min(max(int(wheel_angle*self.steer_ratio), -threshold), threshold))

            lah_viz = LookAheadViz(lah_pt)
            self.pub_lah.publish(lah_viz)

            ### the velocity is m/s, target_v and CS.vEgo is m/s
            accel, brake = self.apid.run(self.target_v / 3.6, CS.vEgo) 
            
            vector3.x = steer
            vector3.y = accel
            vector3.z = brake

            # print("from controller", steer, accel, brake)

        if CS.cruiseState != 1:
            ### for PID test
            # vector3.x = CS.actuators.steer
            vector3.x = 0
            vector3.y = CS.actuators.accel
            vector3.z = CS.actuators.brake

        self.pub_target_actuators.publish(vector3)
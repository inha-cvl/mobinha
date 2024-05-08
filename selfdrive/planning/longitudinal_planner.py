import rospy
import math
import time

from std_msgs.msg import Float32, Int8MultiArray
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker

from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.rviz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
HZ = 10

class LongitudinalPlanner:
    def __init__(self, CP):
        self.lidar_obstacle = None
        self.traffic_light_obstacle = None
        self.can_go_check_tick = -1
        self.can_not_go_check_tick = -1
        self.can_go_timer_start = None
        self.can_go_stored = False
        self.lane_information = None
        self.goal_object = None
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision
        
        self.max_v = CP.maxEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.min_v = CP.minEnableSpeed
        self.target_v = 0
        self.st_param = CP.stParam._asdict()
        self.sl_param = CP.slParam._asdict()

        self.last_error = 0
        self.follow_error = 0
        self.integral = 0
        self.rel_v = 0

        self.closest_tracked = None
        self.closest_untracked = None

        self.right_turn_situation = (0,0)
        self.right_turn_situation_real = (0,0)
        self.stop_time_start = None  # 정지 상태 시작 시간
        self.departure_confirm_time = None  # 출발 확인 시간

        rospy.Subscriber('/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber('/mobinha/perception/traffic_light_obstacle',PoseArray, self.traffic_light_obstacle_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        rospy.Subscriber('/mobinha/planning/goal_information', Pose, self.goal_object_cb)
        rospy.Subscriber('/crosswalkPolygon', Marker, self.crosswalk_cb)
        rospy.Subscriber('/mobinha/planning/right_turn_situation', Int8MultiArray, self.right_turn_situation_cb)
        rospy.Subscriber('/mobinha/planning/right_turn_situation_real', Int8MultiArray, self.right_turn_situation_real_cb)
        self.pub_target_v = rospy.Publisher('/mobinha/planning/target_v', Float32, queue_size=1, latch=True)
        self.pub_traffic_light_marker = rospy.Publisher('/mobinha/planner/traffic_light_marker', Marker, queue_size=1)
        self.pub_accerror = rospy.Publisher('/mobinha/control/accerror', Float32, queue_size=1)

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.z, pose.orientation.x, pose.orientation.y)for pose in msg.poses]

    def traffic_light_obstacle_cb(self, msg):
        self.traffic_light_obstacle = [(pose.position.x, pose.position.y, pose.position.z)for pose in msg.poses]

    def lane_information_cb(self, msg):
        # [0] id, [1] forward_direction, [2] stop line distance [3] forward_curvature
        self.lane_information = [msg.position.x,msg.position.y, msg.position.z, msg.orientation.x]

        if self.lane_information[0] == 979 or self.lane_information[0] == 9:
            self.ref_v = 33
        elif self.lane_information[0] == 982:
            self.ref_v = 38
        else:
            self.ref_v = self.max_v

    def goal_object_cb(self, msg):
        self.goal_object = (msg.position.x, msg.position.y, msg.position.z)

    def crosswalk_cb(self, msg):
        self.crosswalk = [(point.x, point.y) for point in msg.points]

    def right_turn_situation_cb(self, msg):
        self.right_turn_situation = msg.data
    def right_turn_situation_real_cb(self, msg):
        self.right_turn_situation_real = msg.data

    def find_closest_point(self, current_location, waypoints):
        closest_point = None
        min_distance = float('inf')

        for point in waypoints:
            distance = math.sqrt((current_location[0] - point[0])**2 + (current_location[1] - point[1])**2)
            if distance < min_distance:
                min_distance = distance
                closest_point = point

        return min_distance

    def sigmoid_logit_function(self, s):
        if s <=0:
            out = 0
        elif s >= 1:
            out = 1
        else:
            out = ((1+((s*(1-self.sl_param["mu"]))/(self.sl_param["mu"]*(1-s)))**-self.sl_param["v"])**-1).real
        return out

    # static object
    def get_safe_obs_distance_s(self, v_ego, desired_ttc=2, comfort_decel=1.5, offset=5): # cur v = v ego (m/s), 2 sec, 2.5 decel (m/s^2)
        return ((v_ego ** 2) / (2 * comfort_decel) + desired_ttc * v_ego + offset)
    def desired_follow_distance_s(self, v_ego):
        return max(5, self.get_safe_obs_distance_s(v_ego))
    
    # moving object
    def get_stoped_equivalence_factor(self, v_lead, comfort_decel=2):
        if v_lead < 1.9 * KPH_TO_MPS:
            v_lead = 0
        else:
            v_lead = v_lead
        return ((v_lead**2) / (2*comfort_decel))
    def get_safe_obs_distance(self, v_ego, desired_ttc=4, comfort_decel=3, offset=4): # cur v = v ego (m/s), 2 sec, 2.5 decel (m/s^2)
        return ((v_ego ** 2) / (2 * comfort_decel) + desired_ttc * v_ego + offset)
    def desired_follow_distance(self, v_ego, v_lead=0):
        return max(4, self.get_safe_obs_distance(v_ego) - self.get_stoped_equivalence_factor(v_lead)) 

    def get_dynamic_gain(self, error, ttc, kp=0.1/HZ, ki=0.0/HZ, kd=0.08/HZ):
        self.integral += error*(1/HZ)
        self.integral = max(-5, min(self.integral, 5))
        derivative = (error - self.last_error)/(1/HZ) #  frame calculate.
        self.last_error = error
        if error < 0:
            # print("e:",round(error,2),"a:",round(-(kp*error + ki*self.integral + kd*derivative)*HZ,2),"m/s")
            return max(0/HZ, min(1.5/HZ, -(kp*error + ki*self.integral + kd*derivative)))
        elif 0 > ttc > -3:
            # print("warn e:",round(error,2),"a:",round(-(kp*error + ki*self.integral + kd*derivative)*HZ,2),"m/s")
            return min(0/HZ, max(-7/HZ, -(kp*error + ki*self.integral + kd*derivative)))
        else:
            # print("e:",round(error,2),"a:",round(-(kp*error + ki*self.integral + kd*derivative)*HZ,2),"m/s")
            return min(0/HZ, max(-3/HZ, -(kp*error + ki*self.integral + kd*derivative)))
        # TODO: error part 0~-7 0~-3
        
    def get_static_gain(self, error, ttc, gain=0.1/HZ):
        if error < 0:
            return 1.5 / HZ
        elif 0 < ttc < 3:
            return max(0/HZ, min(7/HZ, error*gain))
        else:
            return max(0/HZ, min(4/HZ, error*gain))
        
    def dynamic_consider_range(self, max_v, base_range=50):  # input max_v unit (m/s)#TODO:range ignore check
        return (base_range + (0.267*(max_v)**1.902))*self.M_TO_IDX 

    def get_params(self, max_v, distance):# input distance unit (idx) 
        consider_distance = self.dynamic_consider_range(self.ref_v*KPH_TO_MPS) # consider_distance unit (idx) 
        if 0 < distance < consider_distance:
            norm_s = distance/consider_distance
        elif distance >= consider_distance:
            norm_s = 1
        else:
            norm_s = 0
        min_s = distance*self.IDX_TO_M
        pi = self.sigmoid_logit_function(norm_s)# if 0<norm_s<1 else 1
        target_v = max_v * pi
        return target_v, min_s
    
    def static_velocity_plan(self, cur_v, max_v, static_d):
        target_v, min_s = self.get_params(max_v, static_d) # input static d unit (idx), output min_s unit (m)
        follow_distance = self.desired_follow_distance_s(cur_v) #output follow_distance unit (m)
        ttc = min_s / cur_v if cur_v != 0 else min_s
        self.follow_error = follow_distance-min_s # negative is acceleration. but if min_s is nearby 0, we need deceleration.
        gain = self.get_static_gain(self.follow_error, ttc)
        if self.follow_error < 0: # MINUS is ACCEL
            target_v = min(max_v, self.target_v + gain)
        else: # PLUS is DECEL
            target_v = max(0, self.target_v - gain)
        return target_v

    def dynamic_velocity_plan(self, cur_v, max_v, dynamic_d, v_ego):
        target_v, min_s = self.get_params(max_v, dynamic_d) # input static d unit (idx), output min_s unit (m)
        follow_distance = self.desired_follow_distance(cur_v, self.rel_v + cur_v) #output follow_distance unit (m)
        ttc = min_s / self.rel_v if self.rel_v != 0 else min_s# minus value is collision case
        self.follow_error = follow_distance-min_s
        gain = self.get_dynamic_gain(self.follow_error, ttc)
        if self.follow_error < 0: # MINUS is ACCEL
            if v_ego < 0.4*MPS_TO_KPH and (self.rel_v + cur_v) < 15*KPH_TO_MPS:
                target_v = min(max_v, self.target_v + 0.8/HZ)
            else:
                target_v = min(max_v, self.target_v + gain)
        else: # PLUS is DECEL
            target_v = max(0, self.target_v + gain)

        return target_v

    def traffic_light_to_obstacle(self, traffic_light, forward_direction):
        # stop_list = [[6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13], [
        #     6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13]]
        stop_list = [[7, 8, 9, 10, 14], # straight
                [6, 7, 8, 10, 12, 13, 14], # left
                [7, 8, 9, 10, 12, 14], # right
                [7, 8, 10,  12, 14], # left lane change
                [7, 8, 10,  12, 14], # right lane change
                [6, 7, 8, 10, 12, 14]] # u turn
        if traffic_light in stop_list[forward_direction]:  # Stop Sign
            return False
        else: # Go sign
                return True

    def check_dynamic_objects(self, cur_v, local_s, veh_pose):
        offset = 7.5*self.M_TO_IDX
        dynamic_s = 90*self.M_TO_IDX 
        self.rel_v = 0
        if self.lidar_obstacle is not None:
            for lobs in self.lidar_obstacle:
                if lobs[2] >= -1.45 and lobs[2] <= 1.45:  # object in my lane

                    dynamic_s = lobs[1]-offset-local_s
                    if lobs[4] > 1: # tracking
                        self.rel_v = lobs[3]
                        return dynamic_s
                    else: # only cluster is track_id = 0
                        self.rel_v = 0 # TODO: track and cluster box color modify
                        return dynamic_s
        return dynamic_s
    
    def can_depart(self, current_velocity):
        current_time = time.time()
        if self.departure_confirm_time is not None and current_time - self.departure_confirm_time < 10:
            return True
        if current_velocity < 0.1:
            if self.stop_time_start is None:
                self.stop_time_start = current_time
            else:
                if current_time - self.stop_time_start > 1:
                    self.departure_confirm_time = current_time
                    return True
        else:
            self.stop_time_start = None
            self.departure_confirm_time = None
        return False

###temp

    def check_static_object(self, local_path, local_s, veh_pose, v_ego):
        local_len = len(local_path)
        goal_offset = 1.5*self.M_TO_IDX
        tl_offset = 7*self.M_TO_IDX
        cw_offset = 5*self.M_TO_IDX
        static_s1, static_s2 = 90*self.M_TO_IDX, 90*self.M_TO_IDX
        # [1] = Goal Object
        if self.goal_object is not None:
            left = (self.goal_object[1]-self.goal_object[2]) * self.M_TO_IDX
            if left <= local_len:
                if left-goal_offset < 90*self.M_TO_IDX:
                    static_s1 = left-goal_offset
        # [2] = Traffic Light                
        if self.traffic_light_obstacle is not None:
            # not all stop and only stop front of crosswalk
            # self.right_turn_situation = [0, 0], [0, 1], [1, 0], [1, 1] # [0] = car, [1] = pedestrian
            if self.lane_information[1] == 2: # Right Turn
                min_distance = self.find_closest_point(veh_pose, self.crosswalk)
                
                if self.right_turn_situation == (0,0) and self.right_turn_situation_real == (0,0):
                    static_s2 = min_distance*self.M_TO_IDX-cw_offset
                    if self.can_depart(v_ego):
                        print("can_depart")
                        static_s2 = 90*self.M_TO_IDX
                elif self.right_turn_situation_real == (0,1) or self.right_turn_situation == (0, 1) \
                    or self.right_turn_situation == (1, 0) or self.right_turn_situation == (1,1):
                    static_s2 = min_distance*self.M_TO_IDX-cw_offset
            else:
                can_go = False
                if len(self.traffic_light_obstacle) > 0:
                    tlobs = self.traffic_light_obstacle[0]
                    if self.traffic_light_to_obstacle(int(tlobs[1]), int(self.lane_information[1])):
                        can_go = True
                if not can_go:
                    if self.lane_information[2] < math.inf:
                        left_distance_to_stopline = self.lane_information[2]-tl_offset-local_s
                        if left_distance_to_stopline < 90*self.M_TO_IDX:
                            static_s2 = left_distance_to_stopline
                        if static_s2 < -10*self.M_TO_IDX: # passed traffic light is not considered
                            static_s2 = 90*self.M_TO_IDX
        return min(static_s1, static_s2)

    def run(self, sm, pp=0, local_path=None):
        CS = sm.CS
        lgp = 0
        self.pub_target_v.publish(Float32(self.target_v))
        self.pub_accerror.publish(Float32(self.follow_error))
        if local_path != None and self.lane_information != None:
            local_idx = calc_idx(local_path, (CS.position.x, CS.position.y))
            if CS.cruiseState == 1:
                local_curv_v = calculate_v_by_curvature(self.lane_information, self.ref_v, self.min_v, CS.vEgo) # info, kph, kph, mps
                static_d = self.check_static_object(local_path, local_idx, (CS.position.x, CS.position.y), CS.vEgo) # output unit: idx
                dynamic_d = self.check_dynamic_objects(CS.vEgo, local_idx, (CS.position.x, CS.position.y)) # output unit: idx
                target_v_static = self.static_velocity_plan(CS.vEgo, local_curv_v, static_d)
                target_v_dynamic = self.dynamic_velocity_plan(CS.vEgo, local_curv_v, dynamic_d, CS.vEgo)
                self.target_v = min(target_v_static, target_v_dynamic)
            else:
                self.target_v = CS.vEgo

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.01:
                    lgp = 2
            else:
                lgp = 1

        return lgp

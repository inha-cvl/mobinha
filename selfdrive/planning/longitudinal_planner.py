import rospy
import math
import time

from std_msgs.msg import Float32, Int8MultiArray, Int8, String
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker

from selfdrive.planning.libs.planner_utils import *
from selfdrive.planning.libs.velocity_planner import VELOCITY_PLANNER
from selfdrive.visualize.rviz_utils import *

from selfdrive.planning.libs.map import LaneletMap, TileMap
from path_planner import PathPlanner 

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
HZ = 10

class LongitudinalPlanner:
    def __init__(self, CP):
        self.lmap = LaneletMap(CP.mapParam.path)
        self.tmap = TileMap(self.lmap.lanelets, CP.mapParam.tileSize)
        self.current_blinker_state = (0, None)
        self.turning_flag = None
        self.turng_target_v = -1
        self.hist_target_v = 0
        self.hist_target_flag = None
        self.turning_state = [
        "No turning", # 0 
        # 1           2            3            4              5          6
        "Lchanging", "Rchanging", "will Lturn", "will Rturn", "Lturing", "Rturing",
        # 7               8
        "will Rotary in", "will Rotary out", 
        # 9            10             11               12            13
        "now LPocket", "now RPocket", "will LPocket", "will RPocket", "Keeping"
        ]
        self.start_waiting_time = 0
        self.timeer_count = 0


        self.lidar_obstacle = None
        self.traffic_light_obstacle = None
        self.can_go_check_tick = -1
        self.can_not_go_check_tick = -1
        self.can_go_timer_start = None
        self.can_go_stored = False
        self.lane_information = None
        self.local_id = None
        self.goal_object = None
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision
        
        self.max_v = CP.maxEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.min_v = CP.minEnableSpeed
        self.target_v = 0
        self.st_param = CP.stParam._asdict()
        self.sl_param = CP.slParam._asdict()

        self.velo_pl = VELOCITY_PLANNER(target_velocity = self.max_v, max_velocity = self.max_v, current_velocity = 0)


        self.last_error = 0
        self.follow_error = 0
        self.integral = 0
        self.rel_v = 0

        self.closest_tracked = None
        self.closest_untracked = None

        self.right_turn_situation = (0,0)
        self.right_turn_situation_real = (0,0)
        self.waiting_at_crosswalk = False

        self.hist_yaw = 0

        rospy.Subscriber('/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber('/mobinha/perception/traffic_light_obstacle',PoseArray, self.traffic_light_obstacle_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        rospy.Subscriber('/mobinha/planning/local_id',String, self.local_id_cb)
        ### 2024.05.23 test
        rospy.Subscriber('/mobinha/planning/splited_localid',String, self.splited_local_id_cb)
        ### 2024.05.23 test
        ### 2024.05.24 test
        rospy.Subscriber('/mobinha/planning/turning_flag', String, self.turning_flag_cb)
        rospy.Subscriber('/mobinha/planning/turning_target_v', Int8, self.turning_target_v_cb)
        ### 2024.05.24 test
        rospy.Subscriber('/mobinha/planning/goal_information', Pose, self.goal_object_cb)
        rospy.Subscriber('/mobinha/visualize/max_v', Int8, self.max_v_cb)
        rospy.Subscriber('/crosswalkPolygon', Marker, self.crosswalk_cb)
        rospy.Subscriber('/mobinha/planning/right_turn_situation', Int8MultiArray, self.right_turn_situation_cb)
        rospy.Subscriber('/mobinha/planning/right_turn_situation_real', Int8MultiArray, self.right_turn_situation_real_cb)
        self.pub_target_v = rospy.Publisher('/mobinha/planning/target_v', Float32, queue_size=1, latch=True)
        self.pub_traffic_light_marker = rospy.Publisher('/mobinha/planner/traffic_light_marker', Marker, queue_size=1)
        self.pub_accerror = rospy.Publisher('/mobinha/control/accerror', Float32, queue_size=1)


    ### 2024.05.24 test
    def turning_flag_cb(self, msg):
        self.turning_flag = msg.data
        print(f'here is turing flag {self.turning_flag}')

    def turning_target_v_cb(self, msg):
        self.turng_target_v = msg.data
        print(f'here is turng_target_v {self.turng_target_v}')
    ### 2024.05.24 test

    def max_v_cb(self, msg):
        if msg.data != None:
            self.max_v = msg.data
        else:
            self.max_v = 45

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.z, pose.orientation.x, pose.orientation.y)for pose in msg.poses]

    def traffic_light_obstacle_cb(self, msg):
        self.traffic_light_obstacle = [(pose.position.x, pose.position.y, pose.position.z)for pose in msg.poses]

    def lane_information_cb(self, msg):
        # [0] id, [1] forward_direction, [2] stop line distance [3] forward_curvature
        self.lane_information = [msg.position.x,msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y]

        if self.lane_information[0] == 979 or self.lane_information[0] == 9:
            self.ref_v = 33
        elif self.lane_information[0] == 982:
            self.ref_v = 38
        else:
            self.ref_v = self.max_v

    def local_id_cb(self, msg):
        # [0] id, [1] forward_direction, [2] stop line distance [3] forward_curvature
        self.local_id = msg.data.split(',')

    ### 2024.05.23 test
    def splited_local_id_cb(self, msg):
        # [0] id, [1] forward_direction, [2] stop line distance [3] forward_curvature
        self.splited_local_id = msg.data
        print(f'+++++++++++++++++++++++\n here is splited local id {self.splited_local_id}')

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
            # out = ((1+((s*(1-self.sl_param["mu"]))/(self.sl_param["mu"]*(1-s)))**-self.sl_param["v"])**-1).real
            out = ((1+((s*(0.7))/(.3*(1-s)))**-0.7)**-1).real
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
            return min(0/HZ, max(-8/HZ, -(kp*error + ki*self.integral + kd*derivative)))
        else:
            # print("e:",round(error,2),"a:",round(-(kp*error + ki*self.integral + kd*derivative)*HZ,2),"m/s")
            return min(0/HZ, max(-3/HZ, -(kp*error + ki*self.integral + kd*derivative)))
        # TODO: error part 0~-7 0~-3


    def calculate_v_by_curvature(self, idx, lanelets, ids, hist_yaw,ref_v): # info, kph, kph, mps
        target_v = ref_v 
        ###
        lf = int(idx+20) # 5m ~ 65m
        # left_turn_lane = [1, 2, 91]

        if lf < 0:
            lf = 0
        elif lf > len(ids)-1:
            lf = len(ids)-1
        next_id_1 = ids[lf].split('_')[0]
        # print(f'this is next_id_1 {next_id_1}')
        # print(f'this is now_id {now_id}')

        yaw_temp = lanelets[next_id_1]['yaw'][:30] if len(lanelets[next_id_1]['yaw']) >30 else lanelets[next_id_1]['yaw']
        yaw_variation = max(yaw_temp) - min(yaw_temp)

        if yaw_variation > 0.27:
            target_v = 20  # min_v  #### 15km/h is proofed in k-city

        return target_v   ## -> km/h

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
        print(f'here is sigmoid logic {pi}, max_v {max_v}')
        target_v = max_v * pi 
        return target_v, min_s
    
    def static_velocity_plan(self, cur_v, max_v, static_d):
        ### cur_v MPS
        turnging_timing = self.turning_state[3:9]

        if self.hist_target_flag in turnging_timing:
            max_v = self.hist_target_v
        target_v, min_s = self.get_params(max_v, static_d) # input static d unit (idx), output min_s unit (m)
        
        # if static_d <= (cur_v * KPH_TO_MPS) + 20:
        #     print(f' sucas afsodfj ')
        #     target_v = 0
        # else:
        #     target_v = max_v

        ###Cloudn't understance====xingyou
        # follow_distance = self.desired_follow_distance_s(cur_v) #output follow_distance unit (m)
        # ttc = min_s / cur_v if cur_v != 0 else min_s
        # self.follow_error = follow_distance-min_s # negative is acceleration. but if min_s is nearby 0, we need deceleration.
        # gain = self.get_static_gain(self.follow_error, ttc)
        # if self.follow_error < 0: # MINUS is ACCEL
        #     target_v = min(max_v, self.target_v + gain)
        # else: # PLUS is DECEL
        #     target_v = max(0, self.target_v - gain)

        #print(f'here is the static {target_v}')
        return target_v 

    def dynamic_velocity_plan(self, cur_v, max_v, dynamic_d, v_ego):
        target_v, min_s = self.get_params(max_v, dynamic_d) # input static d unit (idx), output min_s unit (m)

        ###Cloudn't understance====xingyou
        # follow_distance = self.desired_follow_distance(cur_v, self.rel_v + cur_v) #output follow_distance unit (m)
        # ttc = min_s / self.rel_v if self.rel_v != 0 else min_s# minus value is collision case
        # self.follow_error = follow_distance - min_s
        # gain = self.get_dynamic_gain(self.follow_error, ttc)
        # if self.follow_error < 0: # MINUS is ACCEL
        #     if v_ego < 0.4*MPS_TO_KPH and (self.rel_v + cur_v) < 15*KPH_TO_MPS:
        #         target_v = min(max_v, self.target_v + 0.8/HZ)
        #     else:
        #         target_v = min(max_v, self.target_v + gain)
        # else: # PLUS is DECEL
        #     target_v = max(0, self.target_v + gain)

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
                    # dynamic_s = math.sqrt((lobs[5] - veh_pose[0])**2 + (lobs[6] - veh_pose[1])**2) - offset
                    # dynamic_s = round(dynamic_s*self.M_TO_IDX,2)
                    dynamic_s = lobs[1]-offset-local_s
                    if lobs[4] > 1: # tracking
                        self.rel_v = lobs[3]
                        return dynamic_s
                    else: # only cluster is track_id = 0
                        self.rel_v = 0 # TODO: track and cluster box color modify
                        return dynamic_s
        return dynamic_s
    
    ## output is the distance from ego car to stopline 
    def check_static_object(self, local_path, local_s, veh_pose, v_ego): 
        local_len = len(local_path)
        goal_offset = 1.5*self.M_TO_IDX
        tl_offset = 7*self.M_TO_IDX
        cw_offset = 4*self.M_TO_IDX
        static_s1, static_s2 = 90*self.M_TO_IDX, 90*self.M_TO_IDX
        can_go = False
        
        left_distance_to_stopline = self.lane_information[2]-tl_offset-local_s

        print(f'lane_information {self.lane_information[2]}')
        print(f'Distacne {self.lane_information[2]-tl_offset-local_s}')
        print(f'Unit {self.M_TO_IDX}')
        print(f'tl_offset {tl_offset}')
        print(f'local_s {local_s}') 
        
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

            ### judge the traffic light and go directly or stop
            if len(self.traffic_light_obstacle) > 0:
                tlobs = self.traffic_light_obstacle[0]
                if self.traffic_light_to_obstacle(int(tlobs[1]), int(self.lane_information[1])):
                    can_go = True

        right_turn_timimg = [self.turning_state[4], self.turning_state[6], self.turning_state[7], self.turning_state[12]]
        if self.hist_target_flag in right_turn_timimg: # Right Turn
            timeer = time.time()
            min_distance = self.find_closest_point(veh_pose, self.crosswalk)
                
            print(f'Now is right turning')
            if not can_go:
                if self.lane_information[2] < math.inf:
                    left_distance_to_stopline = self.lane_information[2]-tl_offset-local_s
                    if left_distance_to_stopline < 90*self.M_TO_IDX:
                        static_s2 = left_distance_to_stopline
                    if static_s2 < -10*self.M_TO_IDX: # passed traffic light is not considered
                        static_s2 = 90*self.M_TO_IDX
                        self.timeer_count = 0
                        self.start_waiting_time = 0

                if self.right_turn_situation == (0,0):
                    pass
                elif self.right_turn_situation == (0, 1):
                    static_s2 = min_distance*self.M_TO_IDX-cw_offset
                elif self.right_turn_situation == (1, 0):
                    static_s2 = min_distance*self.M_TO_IDX-cw_offset
                elif self.right_turn_situation == (1, 1):
                    static_s2 = min_distance*self.M_TO_IDX-cw_offset
            if v_ego == 0 and static_s2 < 10: ### closer stopline in 10m and ego car stoped 
                if self.timeer_count == 0:
                    self.start_waiting_time = time.time()
                    self.timeer_count +=1

                
            self.time_out = timeer - self.start_waiting_time
            print(f'================================= ')
            print(f'right turning start_waiting_time is {self.start_waiting_time}')
            print(f'right turning time out is {self.time_out}')
            print(f'right turning timeer_count is {self.timeer_count}')
            print(f'self.hist_target_flag is {self.hist_target_flag}')
            print(f'================================= ')
            if self.time_out > 2 and self.time_out < timeer:
                static_s2 = 90*self.M_TO_IDX

        else:
            ### Check distance with spot line to ours => self.lane_information[2] 
            if not can_go:
                if self.lane_information[2] < math.inf:
                    left_distance_to_stopline = self.lane_information[2]-tl_offset-local_s
                    if left_distance_to_stopline < 90*self.M_TO_IDX:
                        static_s2 = left_distance_to_stopline
                    if static_s2 < -10*self.M_TO_IDX: # passed traffic light is not considered
                        static_s2 = 90*self.M_TO_IDX

        return min(static_s1, static_s2)
    
    def temp_judge(self, ):
        pass

    def run(self, sm, pp=0, local_path=None):
        CS = sm.CS

        turnging_timing = self.turning_state[3:9]

        lgp = 0
        self.pub_target_v.publish(Float32(self.target_v))
        self.pub_accerror.publish(Float32(self.follow_error))
        if local_path != None and self.lane_information != None:
            local_idx = calc_idx(local_path, (CS.position.x, CS.position.y))
            if CS.cruiseState == 1:
                if self.turning_flag in turnging_timing and self.turng_target_v != -1:
                    self.hist_target_v = self.turng_target_v
                if self.turning_flag != self.turning_state[13]:## keeping
                    self.hist_target_flag = self.turning_flag

                local_curv_v = self.calculate_v_by_curvature(local_idx, self.lmap.lanelets, self.local_id, self.hist_yaw, self.ref_v) # info, kph, kph, mps, roi path for curve
                static_d = self.check_static_object(local_path, local_idx, (CS.position.x, CS.position.y), CS.vEgo) # output unit: idx
                dynamic_d = self.check_dynamic_objects(CS.vEgo, local_idx, (CS.position.x, CS.position.y)) # output unit: idx


                target_v_static = self.static_velocity_plan(CS.vEgo, local_curv_v, static_d)
                target_v_dynamic = self.dynamic_velocity_plan(CS.vEgo, local_curv_v, dynamic_d, CS.vEgo)
                self.target_v = min(target_v_static, target_v_dynamic)
                print(f'current static_d is {static_d}')
                print(f'current dynamic_d is {dynamic_d}')
                print(f'local_curv_v_d is {local_curv_v}')
                print(f'current target_v_static is {target_v_static}')
                print(f'target_v_dynamic is {target_v_dynamic}')
                print(f'Current target_v is {self.target_v}')

                if (CS.vEgo * MPS_TO_KPH) < 1 and self.target_v == 0:
                    self.target_v = 0
                


            else:
                self.target_v = CS.vEgo * MPS_TO_KPH

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.01:
                    lgp = 2
            else:
                lgp = 1
        return lgp
        

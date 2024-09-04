import rospy
import math
import time

from std_msgs.msg import Float32, Int8MultiArray, Header, String
from geometry_msgs.msg import PoseArray, Pose, Vector3
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
        self.pub_acc_plot = rospy.Publisher('/mobinha/acc_plot', Pose, queue_size=1)
        
        ############ for planner simulation by JM
        from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, ObjectStatusList
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)
        self.ego_pos = [0, 0]
        # self.ego_velocity = [0, 0]

        rospy.Subscriber('/mobinha/planning/stopline_pos', PoseArray, self.stopline_cb)
        self.stopline_point1 = None
        self.stopline_point2 = None
        self.stopline_point1_last = None

        # 신호 받아오는것까지 ok
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.trafficLight_type_cb)
        self.trafficLight_header = None
        self.trafficLight_last_header = None
        self.trafficLight_type = None

        # Estop 완
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.obstacle_pos_cb)
        self.object_list = PoseArray()

        self.stopline_no_traffic_light_timer = rospy.Time.now()
        self.stopline_timer_flag = True
        self.stopline_stopped = False

    def stopline_cb(self, msg):
        self.stopline_point1 = [msg.poses[0].position.x, msg.poses[0].position.y]
        self.stopline_point2 = [msg.poses[1].position.x, msg.poses[1].position.y]
        if self.stopline_point1_last is None:
            self.stopline_point1_last = self.stopline_point1

    def ego_topic_cb(self, msg): # 너무 부정확함 쓰면 안 됨
        self.ego_pos[0] = msg.position.x
        self.ego_pos[1] = msg.position.y
        # self.ego_velocity[0] = msg.velocity.x
        # self.ego_velocity[1] = msg.velocity.y

    def trafficLight_type_cb(self, msg):
        #TODO on simulator: 신호 안 들어올 때 trafficLight_type = None 구현
        self.trafficLight_header = msg.header
        self.trafficLight_type = msg.trafficLightStatus
        if self.trafficLight_last_header is None:
            self.trafficLight_last_header = msg.header

    def obstacle_pos_cb(self, msg):
        object_list = PoseArray()
        for obj in msg.npc_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        for obj in msg.obstacle_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        for obj in msg.pedestrian_list:
            pose = Pose()
            pose.position.x = obj.position.x
            pose.position.y = obj.position.y
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        self.object_list = object_list

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

    def stopline_timer(self, sec):   
        print(sec - ((rospy.Time.now() - self.stopline_no_traffic_light_timer).to_sec()), "(s) left")
        if (rospy.Time.now() - self.stopline_no_traffic_light_timer).to_sec() >= sec:
            result = True
        else:
            result = False

        return result

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

    def check_static_object(self, local_path, local_s, veh_pose, v_ego):
        local_len = len(local_path)
        goal_offset = 1.5*self.M_TO_IDX
        tl_offset = 5*self.M_TO_IDX # origin 7
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
                        #print("can_depart")
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

    def distance_to_stopline(self, ego_pos, ego_heading, stopline_pos):
        print("stopline pos: ", stopline_pos)
        distance_to_stopline = ((ego_pos.x-stopline_pos[0])**2 + (ego_pos.y-stopline_pos[1])**2)**0.5
        heading_vector = (math.cos(ego_heading), math.sin(ego_heading))
        ego_to_stopline = (stopline_pos[0]-ego_pos.x, stopline_pos[1]-ego_pos.y)
        dot_product_value = sum(a * b for a, b in zip(ego_to_stopline, heading_vector))
        norm_v1 = math.sqrt(sum(a * a for a in ego_to_stopline))
        norm_v2 = math.sqrt(sum(b * b for b in heading_vector))
        cos_theta = dot_product_value / (norm_v1 * norm_v2)
        if cos_theta < 0:
            distance_to_stopline = -distance_to_stopline
        else:
            pass
            print("theta: ", math.acos(cos_theta))
        return distance_to_stopline
    
    def safe_intersection_for_morai(self):
        for obs in self.object_list.poses:
            if 23.188 < obs.position.x < 46.253 and 1158.194 < obs.position.y < 1165.082:
                print("DANGEROUS")
                return False
        print("SAFE")
        return True
    
    def safe_roundabout_for_morai(self):
        for obs in self.object_list.poses:
            if 18.571 < obs.position.x < 45.111 and 1834.754 < obs.position.y < 1842.921:
                print("DANGEROUS")
                return False
        print("SAFE")
        return True

    def safe_for_morai(self, ego_pos):
        result = True
        if ((ego_pos.x - 23.188)**2 + (ego_pos.y - 1153.194)**2)**0.5 < 100:
            print("safety check for intersection")
            for obs in self.object_list.poses:
                if 23.188 < obs.position.x < 46.253 and 1153.194 < obs.position.y < 1165.082:
                    result = False
                    print("[intersection] DANGEROUS")
        if ((ego_pos.x - 18.571)**2 + (ego_pos.y - 1834.754)**2)**0.5 < 100:
            print("safety check for roundabout")
            for obs in self.object_list.poses:
                if 18.571 < obs.position.x < 45.111 and 1834.754 < obs.position.y < 1842.921:
                    result = False
                    print("[roundabout] DANGEROUS")

        return result
    
    
    def acc_for_morai(self):
        result = 5
        for obs in self.object_list.poses:
            if ((obs.position.x - self.ego_pos[0])**2 + (obs.position.y - self.ego_pos[1])**2)**0.5 < 12:
                print("obstacle close, target_v = 0")
                result = 0
        
        return result
                
    
                
    def run(self, sm, pp=0, local_path=None):
        CS = sm.CS
        lgp = 0
        # print("CS position: ", (CS.position.x, CS.position.y))
        self.pub_target_v.publish(Float32(self.target_v)) ## 얘만패면됨
        self.pub_accerror.publish(Float32(self.follow_error))
        if local_path != None and self.lane_information != None:
            local_idx = calc_idx(local_path, (CS.position.x, CS.position.y))
            if CS.cruiseState == 1:
                # local_curv_v = calculate_v_by_curvature(self.lane_information, self.ref_v, self.min_v, CS.vEgo) # info, kph, kph, mps
                # static_d = self.check_static_object(local_path, local_idx, (CS.position.x, CS.position.y), CS.vEgo) # output unit: idx
                # dynamic_d = self.check_dynamic_objects(CS.vEgo, local_idx, (CS.position.x, CS.position.y)) # output unit: idx
                # target_v_static = self.static_velocity_plan(CS.vEgo, local_curv_v, static_d)
                # target_v_dynamic = self.dynamic_velocity_plan(CS.vEgo, local_curv_v, dynamic_d, CS.vEgo)
                # self.target_v = min(target_v_static, target_v_dynamic)

                ######## for simulation by JM
                # scenario = "traffic_light"
                # scenario = "obstacle"
                # scenario = "roundabout"
                # scenario = "roundabout+intersection"
                # scenario = "ACC"
                scenario = "stopsign+ACC"

                ## traffic light
                if scenario == "traffic_light":
                    # 모라이 인지 한계: 링크에 신호등 없으면 None 메시지 보내는게 아니라 그냥 메시지를 안 보냄
                    # 따라서 헤더 비교하여 신호등 정보 수신이 없으면 None 반환
                    if not None in [self.trafficLight_header, self.trafficLight_last_header]:
                        last_header_time = self.trafficLight_last_header.stamp.secs + 1e-9*self.trafficLight_last_header.stamp.nsecs
                        now_header_time = self.trafficLight_header.stamp.secs + 1e-9*self.trafficLight_header.stamp.nsecs
                        if last_header_time == now_header_time:
                            self.trafficLight_type = None
                        self.trafficLight_last_header = self.trafficLight_header
                    print("Traffic light sign: ", self.trafficLight_type)
                            

                    # distance 계산
                    distance_to_stopline = float('inf')
                    if not None in [self.stopline_point1, self.stopline_point2]:
                        # distance 크기 계산
                        x0, y0 = CS.position.x, CS.position.y
                        x1, y1 = self.stopline_point1
                        x2, y2 = self.stopline_point2
                        distance_to_stopline = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                        # 지났으면 -부호
                        heading_vector = (math.cos(math.radians(CS.yawRate)), math.sin(math.radians(CS.yawRate)))
                        ego_to_stopline = (x1-x0, y1-y0)
                        dot_product_value = sum(a * b for a, b in zip(ego_to_stopline, heading_vector))
                        norm_v1 = math.sqrt(sum(a * a for a in ego_to_stopline))
                        norm_v2 = math.sqrt(sum(b * b for b in heading_vector))
                        cos_theta = dot_product_value / (norm_v1 * norm_v2)
                        if cos_theta < 0:
                            distance_to_stopline = -distance_to_stopline
                    print("Distance to stopline", distance_to_stopline)

                    # 신호등의 존재를 확인
                    if self.trafficLight_type is None: # TODO on ioniq: 신호등 없는 정지선인 경우
                        # 정지선 근처인지 확인
                        if 0 < distance_to_stopline < 10:
                            # 정지선에 정지했는지 확인
                            if not self.stopline_stopped :
                                self.target_v = 0
                                if CS.vEgo > 0.1: # from Ego_topic vel.x
                                    # self.target_v = 0 # here
                                    print("[No trafficLight] stopping at stopline")
                                else:
                                    if self.stopline_timer_flag:
                                        self.stopline_no_traffic_light_timer = rospy.Time.now()
                                        self.stopline_timer_flag = False
                                    if self.stopline_timer(3.0): 
                                        print("stopline timer running...(3 secs)")
                                        self.stopline_stopped = True
                            else:
                                safe_intersection = self.safe_intersection_for_morai()
                                if safe_intersection:
                                # TODO on ioniq: and 사거리 안전하면 조건 추가 << LiDAR 이용하여 판단
                                    self.target_v = 5
                                    print("[No trafficLight] safe, go at 5")
                                else:
                                    self.target_v = 0
                                    print("[No trafficLight] not safe, wait at 0")

                        else:
                            self.target_v = 5

                    else:
                        if self.trafficLight_type not in [48, 20, 16]: #직좌, 직황, 직
                            if 0 < distance_to_stopline < 10:
                                self.target_v = 0
                            else:
                                self.target_v = 5

                        else:
                            self.target_v = 5

                    # 정지선 위치 바뀌면 (신호등 없는 정지선) local variable 초기화
                    if self.stopline_point1[0] != self.stopline_point1_last[0]:
                        self.stopline_timer_flag = True
                        self.stopline_stopped = False
                        self.stopline_point1_last[0] = self.stopline_point1[0]
                        print("[No trafficLight] Stopline with no traffic light initialized")
                    

                    # self.target_v = min(self.target_v, self.acc_for_morai())
                    print("---------------------------------------------------------")
                
                ## obstacle
                if scenario == "obstacle":
                    min_dist = 100
                    for obs in self.object_list.poses:
                        dist = ((CS.position.x-obs.position.x)**2 + (CS.position.y-obs.position.y)**2)**0.5
                        min_dist = min(dist, min_dist)
                    print("MIN DIST(dog): ", min_dist)
                    if min_dist < 13:
                        self.target_v = 0
                    else:
                        self.target_v = 5
                    print("from long_planner, target_v:", self.target_v)

                ## roundabout
                if scenario == "roundabout":
                    # distance 계산
                    distance_to_stopline = float('inf')
                    if not None in [self.stopline_point1, self.stopline_point2]:
                        # distance 크기 계산
                        x0, y0 = CS.position.x, CS.position.y
                        x1, y1 = self.stopline_point1
                        x2, y2 = self.stopline_point2
                        print("stopline points")
                        print(x1, y1)
                        print(x2, y2)
                        distance_to_stopline = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                        # 지났으면 -부호
                        heading_vector = (math.cos(math.radians(CS.yawRate)), math.sin(math.radians(CS.yawRate)))
                        ego_to_stopline = ((x1+x2)/2-x0, (y1+y2)/2-y0)
                        dot_product_value = sum(a * b for a, b in zip(ego_to_stopline, heading_vector))
                        norm_v1 = math.sqrt(sum(a * a for a in ego_to_stopline))
                        norm_v2 = math.sqrt(sum(b * b for b in heading_vector))
                        cos_theta = dot_product_value / (norm_v1 * norm_v2)
                        if cos_theta < 0:
                            distance_to_stopline = -distance_to_stopline
                    print("Distance to stopline", distance_to_stopline)

                    if 0 < distance_to_stopline < 10:
                        # 정지선에 정지했는지 확인
                        if not self.stopline_stopped :
                            if CS.vEgo > 0.1: # from Ego_topic vel.x
                                self.target_v = 0
                                print("[Roundabout] stopping at stopline")
                            else:
                                if self.stopline_timer_flag:
                                    self.stopline_no_traffic_light_timer = rospy.Time.now()
                                    self.stopline_timer_flag = False
                                if self.stopline_timer(3.0): 
                                    print("stopline timer running...(3 secs)")
                                    self.stopline_stopped = True
                        else:
                            safe_roundabout = self.safe_roundabout_for_morai()
                            if safe_roundabout:
                            # TODO on ioniq: and 사거리 안전하면 조건 추가 << LiDAR 이용하여 판단
                                self.target_v = 5
                                print("[Roundabout] safe, go at 5")
                            else:
                                self.target_v = 0
                                print("[Roundabout] not safe, wait at 0")
                    else:
                        self.target_v = 5

                    # 정지선 위치 바뀌면 (신호등 없는 정지선) local variable 초기화
                    # if self.trafficLight_type is not None:
                    if self.stopline_point1[0] != self.stopline_point1_last[0]:
                        self.stopline_timer_flag = True
                        self.stopline_stopped = False
                        self.stopline_point1_last[0] = self.stopline_point1[0]
                        print("[No trafficLight] Stopline with no traffic light initialized")
                    

                    self.target_v = min(self.target_v, self.acc_for_morai())
                    print("---------------------------------------------------------")

                ## roundabout
                if scenario == "roundabout+intersection":
                    # 신호등 정보 처리
                    if not None in [self.trafficLight_header, self.trafficLight_last_header]:
                        last_header_time = self.trafficLight_last_header.stamp.secs + 1e-9*self.trafficLight_last_header.stamp.nsecs
                        now_header_time = self.trafficLight_header.stamp.secs + 1e-9*self.trafficLight_header.stamp.nsecs
                        if last_header_time == now_header_time:
                            self.trafficLight_type = None
                        self.trafficLight_last_header = self.trafficLight_header
                    print("Traffic light sign: ", self.trafficLight_type)

                    # distance 계산
                    distance_to_stopline = float('inf')
                    if not None in [self.stopline_point1, self.stopline_point2]:
                        # distance 크기 계산
                        x0, y0 = CS.position.x, CS.position.y
                        x1, y1 = self.stopline_point1
                        x2, y2 = self.stopline_point2
                        distance_to_stopline = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                        # 지났으면 -부호
                        heading_vector = (math.cos(math.radians(CS.yawRate)), math.sin(math.radians(CS.yawRate)))
                        ego_to_stopline = ((x1+x2)/2-x0, (y1+y2)/2-y0)
                        dot_product_value = sum(a * b for a, b in zip(ego_to_stopline, heading_vector))
                        norm_v1 = math.sqrt(sum(a * a for a in ego_to_stopline))
                        norm_v2 = math.sqrt(sum(b * b for b in heading_vector))
                        cos_theta = dot_product_value / (norm_v1 * norm_v2)
                        if cos_theta < 0:
                            distance_to_stopline = -distance_to_stopline
                    print("Distance to stopline", distance_to_stopline)
                    
                    
                    
                    
                    if self.trafficLight_type is None: # TODO on ioniq: 신호등 없는 정지선인 경우
                        # 정지선 근처인지 확인
                        if 0 < distance_to_stopline < 10:
                            # 정지선에 정지했는지 확인
                            if not self.stopline_stopped:
                                if CS.vEgo > 0.1: # from Ego_topic vel.x
                                    self.target_v = 0
                                    print("[integrated] stopping at stopline")
                                else:
                                    if self.stopline_timer_flag:
                                        self.stopline_no_traffic_light_timer = rospy.Time.now()
                                        self.stopline_timer_flag = False
                                    if self.stopline_timer(3.0): 
                                        print("stopline timer running...(3 secs)")
                                        self.stopline_stopped = True
                            else:
                                safe_intersection = self.safe_for_morai(CS.position)
                                if safe_intersection:
                                # TODO on ioniq: and 사거리 안전하면 조건 추가 << LiDAR 이용하여 판단
                                    self.target_v = 5
                                    print("[integrated] safe, go at 5")
                                else:
                                    self.target_v = 0
                                    print("[integrated] not safe, wait at 0")

                        else:
                            self.target_v = 5

                    elif self.trafficLight_type not in [48, 20, 16]: #직좌, 직황, 직
                        if 0 < distance_to_stopline < 10:
                            self.target_v = 0
                        else:
                            self.target_v = 5

                    else:
                        self.target_v = 5

                    # 정지선 위치 바뀌면 (신호등 없는 정지선) local variable 초기화
                    # if self.trafficLight_type is not None:
                    if self.stopline_point1[0] != self.stopline_point1_last[0]:
                        self.stopline_timer_flag = True
                        self.stopline_stopped = False
                        self.stopline_point1_last[0] = self.stopline_point1[0]
                        print("[integrated] Stopline with no traffic light initialized")
                    

                    # self.target_v = min(self.target_v, self.acc_for_morai())
                    print("---------------------------------------------------------")
            
                ## ACC
                if scenario == "ACC":
                    self.target_v = 50/3.6

                    # safety_distance = max(CS.vEgo*3.6-15, 10) # safe_distance
                    safety_distance = max(CS.vEgo*3.6-15, 9) # safe_distance
                    margin = 0
                    margined_safety_distance = safety_distance + margin

                    nearest_s = 100
                    obs_velocity = 100
                    for obs in self.object_list.poses:
                        current_s = ((obs.position.x - self.ego_pos[0])**2 + (obs.position.y - self.ego_pos[1])**2)**0.5
                        if current_s < nearest_s:
                            nearest_s = current_s
                        
                        obs_velocity = obs.orientation.w/3.6
                    
                    if nearest_s < margined_safety_distance*0.9:
                        status = "danger_zone"
                    elif margined_safety_distance*0.9 < nearest_s < margined_safety_distance*1.4:
                        status = "safe_zone"
                    elif margined_safety_distance*1.4 < nearest_s:
                        status = "far_zone"
                    else:
                        status = "zone_error"
                    print(f"======{status}=======\ncurrent s is {nearest_s:.2f}\ntarget s is {margined_safety_distance:.2f}")
                    
                    
                    s_ratio = nearest_s / margined_safety_distance
                    
                    if status == "danger_zone":
                        # val = obs_velocity*s_ratio
                        val = 10/3.6/21*(current_s - 9)
                    elif status == "safe_zone":
                        val = obs_velocity*s_ratio
                        # val = 10/3.6/21*(current_s - 3)
                    elif status == "far_zone":
                        val = 10 # from curvature
                        # val = obs_velocity*s_ratio**1.5
                        # val = 10/3.6/21*(current_s - 3)
                    else:
                        print("error on status decision: test.py")
                    
                    self.target_v = val
                    print(f"current v is {CS.vEgo:.2f}\ntarget v is {self.target_v:.2f}")
                    
                    acc_plot_msg = Pose()
                    acc_plot_msg.orientation.x = margined_safety_distance
                    acc_plot_msg.orientation.y = current_s
                    acc_plot_msg.orientation.z = self.target_v
                    acc_plot_msg.orientation.w = CS.vEgo
                    self.pub_acc_plot.publish(acc_plot_msg)

                ## stopsign+ACC
                if scenario == "stopsign+ACC":
                    # traffic_light part
                    # 신호등 정보 처리
                    if not None in [self.trafficLight_header, self.trafficLight_last_header]:
                        last_header_time = self.trafficLight_last_header.stamp.secs + 1e-9*self.trafficLight_last_header.stamp.nsecs
                        now_header_time = self.trafficLight_header.stamp.secs + 1e-9*self.trafficLight_header.stamp.nsecs
                        if last_header_time == now_header_time:
                            self.trafficLight_type = None
                        self.trafficLight_last_header = self.trafficLight_header
                    print("Traffic light sign: ", self.trafficLight_type)

                    # distance 계산
                    distance_to_stopline = float('inf')
                    if not None in [self.stopline_point1, self.stopline_point2]:
                        # distance 크기 계산
                        x0, y0 = CS.position.x, CS.position.y
                        x1, y1 = self.stopline_point1
                        x2, y2 = self.stopline_point2
                        distance_to_stopline = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                        # 지났으면 -부호
                        heading_vector = (math.cos(math.radians(CS.yawRate)), math.sin(math.radians(CS.yawRate)))
                        ego_to_stopline = ((x1+x2)/2-x0, (y1+y2)/2-y0)
                        dot_product_value = sum(a * b for a, b in zip(ego_to_stopline, heading_vector))
                        norm_v1 = math.sqrt(sum(a * a for a in ego_to_stopline))
                        norm_v2 = math.sqrt(sum(b * b for b in heading_vector))
                        cos_theta = dot_product_value / (norm_v1 * norm_v2)
                        if cos_theta < 0:
                            distance_to_stopline = -distance_to_stopline
                    print("Distance to stopline", distance_to_stopline)
                    
                    
                    
                    
                    if self.trafficLight_type is None: # TODO on ioniq: 신호등 없는 정지선인 경우
                        # 정지선 근처인지 확인
                        # if 0 < distance_to_stopline < 10:
                            # 정지선에 정지했는지 확인
                        if not self.stopline_stopped:
                            if 0 < distance_to_stopline < max(CS.vEgo*3.6-15, 11):
                                target_v_TL = 10/3.6/21*(distance_to_stopline - 11)
                                if CS.vEgo > 0.02: # from Ego_topic vel.x
                                    print("[integrated] stopping at stopline")
                                else:
                                    if self.stopline_timer_flag:
                                        self.stopline_no_traffic_light_timer = rospy.Time.now()
                                        self.stopline_timer_flag = False
                                    if self.stopline_timer(3.0): 
                                        print("stopline timer running...(3 secs)")
                                        self.stopline_stopped = True
                            else:
                                target_v_TL = 5

                            
                        else:
                            safe_intersection = self.safe_for_morai(CS.position)
                            if safe_intersection:
                            # TODO on ioniq: and 사거리 안전하면 조건 추가 << LiDAR 이용하여 판단
                                target_v_TL = 5
                                print("[integrated] safe, go at 5")
                            else:
                                target_v_TL = 0
                                print("[integrated] not safe, wait at 0")

                        # else:
                        #     target_v_TL = 5

                    elif self.trafficLight_type not in [48, 20, 16]: #직좌, 직황, 직
                        if 0 < distance_to_stopline < 10:
                            target_v_TL = 0
                        else:
                            target_v_TL = 5

                    else:
                        target_v_TL = 5

                    # 정지선 위치 바뀌면 (신호등 없는 정지선) local variable 초기화
                    # if self.trafficLight_type is not None:
                    if self.stopline_point1[0] != self.stopline_point1_last[0]:
                        self.stopline_timer_flag = True
                        self.stopline_stopped = False
                        self.stopline_point1_last[0] = self.stopline_point1[0]
                        print("[integrated] Stopline with no traffic light initialized")
                    
                    # ACC part
                    safety_distance = max(CS.vEgo*3.6-15, 9) # safe_distance
                    margin = 0
                    margined_safety_distance = safety_distance + margin

                    nearest_s = 200
                    current_s = 200
                    obs_velocity = 100
                    for obs in self.object_list.poses:
                        current_s = ((obs.position.x - self.ego_pos[0])**2 + (obs.position.y - self.ego_pos[1])**2)**0.5
                        if current_s < nearest_s:
                            nearest_s = current_s
                        
                        obs_velocity = obs.orientation.w/3.6
                    
                    if nearest_s < margined_safety_distance*0.9:
                        status = "danger_zone"
                    elif margined_safety_distance*0.9 < nearest_s < margined_safety_distance*1.4:
                        status = "safe_zone"
                    elif margined_safety_distance*1.4 < nearest_s:
                        status = "far_zone"
                    else:
                        status = "zone_error"
                    print(f"======{status}=======\ncurrent s is {nearest_s:.2f}\ntarget s is {margined_safety_distance:.2f}")
                    
                    
                    s_ratio = nearest_s / margined_safety_distance
                    
                    if status == "danger_zone":
                        target_v_ACC = 10/3.6/21*(current_s - 9)
                    elif status == "safe_zone":
                        target_v_ACC = obs_velocity*s_ratio
                    elif status == "far_zone":
                        target_v_ACC = 10
                    else:
                        print("error on status decision")
                    
                    print(f"curvature: {10:.2f}\ntraffic_light: {target_v_TL:.2f}\nACC: {target_v_ACC:.2f}")
                    self.target_v = min(10, target_v_TL, target_v_ACC) # 10 is for curavature module
                    print(f"current v is {CS.vEgo:.2f}\ntarget v is {self.target_v:.2f}")
                    
                    acc_plot_msg = Pose()
                    acc_plot_msg.orientation.x = margined_safety_distance
                    acc_plot_msg.orientation.y = current_s
                    acc_plot_msg.orientation.z = self.target_v
                    acc_plot_msg.orientation.w = CS.vEgo
                    self.pub_acc_plot.publish(acc_plot_msg)

                    
                    

            
            else:
                self.target_v = CS.vEgo

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.01:
                    lgp = 2
            else:
                lgp = 1

        return lgp

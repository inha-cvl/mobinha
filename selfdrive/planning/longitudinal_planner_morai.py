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
        self.lane_information = None
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision
        
        self.max_v = CP.maxEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.min_v = CP.minEnableSpeed
        self.target_v = 0
        self.st_param = CP.stParam._asdict()
        self.sl_param = CP.slParam._asdict()

        self.last_error = 0
        self.integral = 0
        self.rel_v = 0

        self.closest_tracked = None
        self.closest_untracked = None

        self.right_turn_situation = (0,0)
        self.right_turn_situation_real = (0,0)
        self.stop_time_start = None  # 정지 상태 시작 시간
        self.departure_confirm_time = None  # 출발 확인 시간

        rospy.Subscriber('/mobinha/planning/lane_information',Pose, self.lane_information_cb)
        rospy.Subscriber('/crosswalkPolygon', Marker, self.crosswalk_cb)
        self.pub_target_v = rospy.Publisher('/mobinha/planning/target_v', Float32, queue_size=1, latch=True)
        self.pub_accerror = rospy.Publisher('/mobinha/control/accerror', Float32, queue_size=1)
        
        ############ for planner simulation by JM
        from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, ObjectStatusList
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)
        self.ego_pos = [0, 0]


         # Stopline
        rospy.Subscriber('/mobinha/planning/stopline_pos', PoseArray, self.stopline_cb)
        self.stopline_point1 = None
        self.stopline_point2 = None
        self.stopline_point1_last = None

        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.trafficLight_type_cb)
        self.trafficLight_header = None
        self.trafficLight_last_header = None
        self.trafficLight_type = None

        self.stopline_timer_start_time = rospy.Time.now()
        self.stopline_timer_flag = True
        self.stopline_stopped = False

        # ACC
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.obstacle_pos_cb)
        self.object_list = PoseArray()

        


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

    def lane_information_cb(self, msg):
        # [0] id, [1] forward_direction, [2] stop line distance [3] forward_curvature
        self.lane_information = [msg.position.x,msg.position.y, msg.position.z, msg.orientation.x]

        if self.lane_information[0] == 979 or self.lane_information[0] == 9:
            self.ref_v = 33
        elif self.lane_information[0] == 982:
            self.ref_v = 38
        else:
            self.ref_v = self.max_v

    def crosswalk_cb(self, msg): # not used yet
        self.crosswalk = [(point.x, point.y) for point in msg.points]

    #### STOPLINE
    def stopline_cb(self, msg):
        self.stopline_point1 = [msg.poses[0].position.x, msg.poses[0].position.y]
        self.stopline_point2 = [msg.poses[1].position.x, msg.poses[1].position.y]
        if self.stopline_point1_last is None:
            self.stopline_point1_last = self.stopline_point1

    def stopline_timer(self, sec):   
        print(sec - ((rospy.Time.now() - self.stopline_timer_start_time).to_sec()), "(s) left")
        if (rospy.Time.now() - self.stopline_timer_start_time).to_sec() >= sec:
            result = True
        else:
            result = False

        return result

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

        return distance_to_stopline
    
    def safe_intersection_for_morai(self): # integrated: safe_for_morai
        for obs in self.object_list.poses:
            if 23.188 < obs.position.x < 46.253 and 1158.194 < obs.position.y < 1165.082:
                print("DANGEROUS")
                return False
        print("SAFE")
        return True
    
    def safe_roundabout_for_morai(self): # integrated: safe_for_morai
        for obs in self.object_list.poses:
            if 18.571 < obs.position.x < 45.111 and 1834.754 < obs.position.y < 1842.921:
                print("DANGEROUS")
                return False
        print("SAFE")
        return True

    def safe_for_morai(self, ego_pos): # 얘가 시나리오별로 다른 ROI를 설정하도록 해야함
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
    
    #### ACC
    def acc_for_morai(self): # need to be changed
        result = 5
        for obs in self.object_list.poses:
            if ((obs.position.x - self.ego_pos[0])**2 + (obs.position.y - self.ego_pos[1])**2)**0.5 < 12:
                print("obstacle close, target_v = 0")
                result = 0
        
        return result
                
    def run(self, sm, pp=0, local_path=None):
        CS = sm.CS
        lgp = 0
        print("CS position: ", (CS.position.x, CS.position.y))
        self.pub_target_v.publish(Float32(self.target_v)) ## 얘만패면됨
        if local_path != None and self.lane_information != None:
            local_idx = calc_idx(local_path, (CS.position.x, CS.position.y))
            if CS.cruiseState == 1:

                ######## for simulation by JM
                # scenario = "traffic_light"
                # scenario = "obstacle"
                # scenario = "roundabout"
                scenario = "roundabout+intersection"

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

                    # 신호에 따라
                    if self.trafficLight_type is None: # TODO on ioniq: 신호등 없는 정지선인 경우
                        # 정지선 근처인지 확인
                        if 0 < distance_to_stopline < 10:
                            # 정지선에 정지했는지 확인
                            if not self.stopline_stopped :
                                if CS.vEgo > 0.1: # from Ego_topic vel.x
                                    self.target_v = 0
                                    print("[No trafficLight] stopping at stopline")
                                else:
                                    if self.stopline_timer_flag:
                                        self.stopline_timer_start_time = rospy.Time.now()
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

                    elif self.trafficLight_type not in [48, 20, 16]: #직좌, 직황, 직
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
                    

                    self.target_v = min(self.target_v, self.acc_for_morai())
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
                                    self.stopline_timer_start_time = rospy.Time.now()
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
                                        self.stopline_timer_start_time = rospy.Time.now()
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
            else:
                self.target_v = CS.vEgo

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.01:
                    lgp = 2
            else:
                lgp = 1

        return lgp

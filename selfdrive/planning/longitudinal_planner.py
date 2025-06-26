import rospy
import math
import time

from std_msgs.msg import Float32, Int8MultiArray, Int8, String
from geometry_msgs.msg import PoseArray, Pose, Point, Polygon
from visualization_msgs.msg import Marker
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, ObjectStatusList

from selfdrive.planning.libs.planner_utils import *
# from selfdrive.planning.libs.velocity_planner import VELOCITY_PLANNER
from selfdrive.visualize.rviz_utils import *

from selfdrive.planning.libs.map import LaneletMap, TileMap
from path_planner import PathPlanner 

import shapely as sh

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
HZ = 10

class LongitudinalPlanner:
    def __init__(self, CP):
        self.lidar_obstacle = None
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision
        
        self.max_v = CP.maxEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.min_v = CP.minEnableSpeed
        self.target_v = 0
        self.st_param = CP.stParam._asdict()
        self.sl_param = CP.slParam._asdict()

        self.follow_error = 0 # for what ?

        # rospy.Subscriber('/mobinha/planning/goal_information', Pose, self.goal_object_cb)
        self.pub_target_v = rospy.Publisher('/mobinha/planning/target_v', Float32, queue_size=1, latch=True)
        self.pub_accerror = rospy.Publisher('/mobinha/control/accerror', Float32, queue_size=1)
        self.pub_acc_plot = rospy.Publisher('/mobinha/acc_plot', Pose, queue_size=1)
        
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)
        self.ego_pos = [0, 0]
        self.transformed_ego_pos = [0, 0]

        rospy.Subscriber('/mobinha/planning/stopline', Marker, self.stopline_cb)
        self.stopline_point1 = None
        self.stopline_point2 = None
        self.stopline_point1_last = None

        rospy.Subscriber('/mobinha/planning/crosswalk_pos', Polygon, self.crosswalk_cb)
        self.crosswalk_polygon = None

        # for morai
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.trafficLight_type_morai_cb)
        # for ioniq
        rospy.Subscriber('/mobinha/perception/camera/bounding_box',PoseArray, self.trafficLight_type_cb)
        self.trafficLight_header = None
        self.trafficLight_last_header = None
        self.trafficLight_type = None

        # for morai
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.obstacle_pos_morai_cb)
        # for ioniq
        rospy.Subscriber('/mobinha/perception/lidar_obstacle', PoseArray, self.obstacle_pos_cb)
        self.object_list = PoseArray()


        self.stopline_no_traffic_light_timer = rospy.Time.now()
        self.stopline_timer_flag = True
        self.stopline_stopped = False
        self.distance_to_stopline = 999
        
        self.now_scenario = '-'
        self.next_scenario = '-'
        self.roi_safe_to_go = False
        self.rightTurn = False
        
        
        
    # callback from path_planner
    def stopline_cb(self, msg):
        self.stopline_point1 = [msg.points[0].x, msg.points[0].y]
        self.stopline_point2 = [msg.points[1].x, msg.points[1].y]
        if self.stopline_point1_last is None:
            self.stopline_point1_last = self.stopline_point1
        print("stopline", self.stopline_point1, self.stopline_point2)
            
    def crosswalk_cb(self, msg):
        points = []
        for pt in msg.points:
            points.append((pt.x, pt.y))
        
        if len(points) > 2:
            self.crosswalk_polygon = sh.Polygon(points)
        else:
            self.crosswalk_polygon = None
    # ------------------------------------------
    
    # callback from MORAI   
    def transform_point(self, morai_point):
        x, y = morai_point
        cos_theta, sin_theta, tx, ty = (0.9997685974969953, 0.023521693953422407, 0.08183366200224204, -0.13516831435460583)
        x_prime = cos_theta * x - sin_theta * y + tx
        y_prime = sin_theta * x + cos_theta * y + ty
        return (x_prime, y_prime)

    def ego_topic_cb(self, msg):
        self.ego_pos[0] = msg.position.x
        self.ego_pos[1] = msg.position.y
        
        self.transformed_ego_pos = self.transform_point(self.ego_pos)
        # self.ego_velocity[0] = msg.velocity.x
        # self.ego_velocity[1] = msg.velocity.y

    def trafficLight_type_morai_cb(self, msg):
        #TODO on simulator: 신호 안 들어올 때 trafficLight_type = None 구현
        self.trafficLight_header = msg.header
        self.trafficLight_type = msg.trafficLightStatus
        if self.trafficLight_last_header is None:
            self.trafficLight_last_header = msg.header

    def obstacle_pos_morai_cb(self, msg):
        object_list = PoseArray()
        for obj in msg.npc_list:
            pose = Pose()
            transformed = self.transform_point((obj.position.x, obj.position.y))
            # transformed = (obj.position.x, obj.position.y)
            pose.position.x = transformed[0]
            pose.position.y = transformed[1]
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
            
        for obj in msg.obstacle_list:
            pose = Pose()
            transformed = self.transform_point((obj.position.x, obj.position.y))
            pose.position.x = transformed[0]
            pose.position.y = transformed[1]
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
            
        for obj in msg.pedestrian_list:
            pose = Pose()
            transformed = self.transform_point((obj.position.x, obj.position.y))
            pose.position.x = transformed[0]
            pose.position.y = transformed[1]
            pose.position.z = obj.heading
            pose.orientation.w = obj.velocity.x
            object_list.poses.append(pose)
        self.object_list = object_list
    # ------------------------------------------

    def obstacle_pos_cb(self, msg):
        object_list = PoseArray()   
        for obj in msg.poses:
            pose = Pose()
            if obj.position.y < 0:
                continue
            pose.position.y = obj.position.y  # rel position
            pose.orientation.w = obj.orientation.w  # rel velocity
            object_list.poses.append(pose)

        print(f"{len(msg.poses)} objects incoming")
        # rel position 기준 정렬
        object_list.poses.sort(key=lambda p: p.position.z)
        self.object_list = object_list

    def trafficLight_type_cb(self, msg): # 마지막으로 들어온 신호등 객체 정보: TODO 인지단에서 주는 확실한 정보로 변경
        '''
        self.tl_list = {
            self.red_3:         7,
            self.yellow_3:      8,
            self.green_3:       13,

            self.red_4:         7,     
            self.yellow_4:      8,
            self.green_4:       13,
            
            self.red_yellow_4:  10,
            self.red_arrow_4:   9,
            self.arrow_green_4: 11
        }
        '''
        self.trafficLight_header = msg.header
        for pose in msg.poses:
            cls, size, prob = pose.position.x, pose.position.y, pose.position.z
            self.trafficLight_type = cls
        if self.trafficLight_last_header is None:
            self.trafficLight_last_header = msg.header
    # ------------------------------------------


    # perception utils
    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.z, pose.orientation.x, pose.orientation.y)for pose in msg.poses]

    # def goal_object_cb(self, msg):
    #     self.goal_object = (msg.position.x, msg.position.y, msg.position.z)
    
    def traffic_light_postprocess(self): # 신호등 정보 후처리
        if not None in [self.trafficLight_header, self.trafficLight_last_header]:
            last_header_time = self.trafficLight_last_header.stamp.secs + 1e-9*self.trafficLight_last_header.stamp.nsecs
            now_header_time = self.trafficLight_header.stamp.secs + 1e-9*self.trafficLight_header.stamp.nsecs
            if last_header_time == now_header_time:
                self.trafficLight_type = None
            self.trafficLight_last_header = self.trafficLight_header
    # ------------------------------------------


    # stopline module utils
    def stopline_timer(self, sec):
        time_to_go = sec - ((rospy.Time.now() - self.stopline_no_traffic_light_timer).to_sec())
        if (rospy.Time.now() - self.stopline_no_traffic_light_timer).to_sec() >= sec:
            result = True
        else:
            result = False

        return result, time_to_go

    def set_distance_to_stopline(self, CS):
        distance_to_stopline = 500
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
            
            self.distance_to_stopline = distance_to_stopline
    # ------------------------------------------
    
    
    # scenario utils
    def is_next_lane_converged(self, lmap, lane_id, global_ids):
        suc_path_id = global_ids[global_ids.index(lane_id) + 1]
        if len(lmap.lanelets[suc_path_id]['predecessor'])>1:
            return True
        return False
    
    def is_any_adjacent_with_same_suc(self, lmap, lane_id):
        suc_id_list = lmap.lanelets[lane_id]['successor']   
        adjacentLeft_id = lmap.lanelets[lane_id]['adjacentLeft'] 
        adjacentRight_id = lmap.lanelets[lane_id]['adjacentRight']

        if len(suc_id_list)==1:
            pre_of_suc_id_list = lmap.lanelets[suc_id_list[0]]['predecessor']
            if adjacentLeft_id in pre_of_suc_id_list or adjacentRight_id in pre_of_suc_id_list:
                return True
        return False
    # ------------------------------------------


    def set_scenario_and_roi(self, lmap, my_lane_id, global_ids): # Roundabout, Merge, Intersection을 남은 path에 따라 결정
        # safety check ROI 설정
        self.merge_roi_ids_waypoints = []
        self.inter_roi_ids_waypoints = []
        self.round_roi_ids_waypoints = []
        self.now_scenario = "----"
        self.next_scenario = "----"
        
        
        if lmap.lanelets[my_lane_id]['roundabout']:
            self.now_scenario = "ROUNDABOUT"
            print("now: roundabout")
            
        if len(global_ids) > global_ids.index(my_lane_id)+1:
            next_path_id = global_ids[global_ids.index(my_lane_id) + 1]
            if self.is_next_lane_converged(lmap, my_lane_id, global_ids):
                if self.is_any_adjacent_with_same_suc(lmap, my_lane_id):
                    self.now_scenario = "MERGE"
                    self.merge_roi_ids_waypoints = self.get_merge_roi(lmap, my_lane_id)
                else:
                    self.now_scenario = "INTERSECTION" # 내 차량 우선: roi X
                    
        if len(global_ids) > global_ids.index(my_lane_id)+2:
            next_path_id = global_ids[global_ids.index(my_lane_id) + 1]
            if self.is_next_lane_converged(lmap, next_path_id, global_ids):
                if self.is_any_adjacent_with_same_suc(lmap, next_path_id):
                    self.next_scenario = "MERGE"
                else:
                    if lmap.lanelets[next_path_id]['roundabout']:
                        self.next_scenario = "ROUNDABOUT"
                        if self.now_scenario != "ROUNDABOUT":
                            self.round_roi_ids_waypoints = self.get_roundabout_roi(lmap, my_lane_id, global_ids)
                    else:
                        self.next_scenario = "INTERSECTION"
                        self.inter_roi_ids_waypoints = self.get_intsection_roi(lmap, my_lane_id, global_ids)
            
            
                    
        print(f"====SCENARIO===\n- NOW: {self.now_scenario}\n- NEXT: {self.next_scenario}\n")
    
    # merge module utils
    def get_merge_roi(self, lmap, lane_id): # adjacentLeft + adjacentRight
        roi_ids_waypoints = []
        adjacentLeft_id = lmap.lanelets[lane_id]['adjacentLeft'] 
        adjacentRight_id = lmap.lanelets[lane_id]['adjacentRight']
        if adjacentLeft_id is not None:
            waypoints_length = len(lmap.lanelets[adjacentLeft_id]['waypoints'])
            roi_ids_waypoints.append((adjacentLeft_id, lmap.lanelets[adjacentLeft_id]['waypoints'][max(waypoints_length-30, 0):waypoints_length-1])) # precision: 1

        if adjacentRight_id is not None:
            waypoints_length = len(lmap.lanelets[adjacentRight_id]['waypoints'])
            roi_ids_waypoints.append((adjacentRight_id, lmap.lanelets[adjacentRight_id]['waypoints'][max(waypoints_length-30, 0):waypoints_length-1])) # precision: 1
        
        return roi_ids_waypoints
    
    def merge_safe_to_go(self, CS):
        safe_to_go = True
        obs_link = ''
        min_lane_dis = 999
        for obs in self.object_list.poses:
            # Merge ROI
            for id_, pts in self.merge_roi_ids_waypoints:
                for pt in pts:
                    dis = ((obs.position.x - pt[0])**2 + (obs.position.y - pt[1])**2)**0.5
                    if dis < 1:
                        safe_to_go = False
                        tmp = id_+" "
                        obs_link += tmp
                        break
                # 내 차량이 진입 시도한 이후에는 우선권이 내 차량에 있음
                for pt in pts:
                    lane_dis = ((pt[0] - CS.position.x)**2 + (pt[1] - CS.position.y)**2)**0.5
                    min_lane_dis = min(lane_dis, min_lane_dis)
                    str1 = "- Min dis to roi lane(for merge): {min_lane_dis}\n"
                    if lane_dis < 3:
                        str2 = "- Min dis < 3, ego go first\n"
                        safe_to_go = True
                        break
        
        return safe_to_go
    # ------------------------------------------
    
    
    # intersection module utils
    def get_intsection_roi(self, lmap, lane_id, global_ids): # ego lane을 제외한 suc^2의 pre
        roi_ids_waypoints = []
        suc_suc_path_id = global_ids[global_ids.index(lane_id) + 2]
        pre_suc_suc_id_list = lmap.lanelets[suc_suc_path_id]['predecessor'].copy()
        
        next_lane_id = global_ids[global_ids.index(lane_id) + 1]
        if next_lane_id in pre_suc_suc_id_list:
            pre_suc_suc_id_list.remove(next_lane_id)
        for id_ in pre_suc_suc_id_list:
            waypoints_length = len(lmap.lanelets[id_]['waypoints'])
            roi_ids_waypoints.append((id_, lmap.lanelets[id_]['waypoints'][max(waypoints_length-13, 0):waypoints_length-1])) # precision: 1, so roi 5~50m
        
        return roi_ids_waypoints
    
    def intersection_safe_to_go(self):
        safe_to_go = True
        obs_link = ''
        for obs in self.object_list.poses:
        # Intersection ROI
            for id_, pts in self.inter_roi_ids_waypoints:
                for pt in pts:
                    dis = ((obs.position.x - pt[0])**2 + (obs.position.y - pt[1])**2)**0.5
                    if dis < 1:
                        safe_to_go = False
                        tmp = id_+" "
                        obs_link += tmp
                        break
        
        return safe_to_go
    # ------------------------------------------
    
    
    # roundabout module utils
    def get_roundabout_roi(self, lmap, lane_id, global_ids): # ego lane을 제외한 suc^2의 pre + pre^2 + pre^3
        roi_ids_waypoints = []
        
        suc_suc_path_id = global_ids[global_ids.index(lane_id) + 2]
        pre_suc_suc_id_list = lmap.lanelets[suc_suc_path_id]['predecessor'].copy()
        next_lane_id = global_ids[global_ids.index(lane_id) + 1]
        if next_lane_id in pre_suc_suc_id_list:
            pre_suc_suc_id_list.remove(next_lane_id)
        
        pre_pre_suc_suc_id_list = []
        for id_ in pre_suc_suc_id_list:
            pre_pre_suc_suc_id_list += lmap.lanelets[id_]['predecessor'].copy()

        pre_pre_pre_suc_suc_id_list = []
        for id_ in pre_pre_suc_suc_id_list:
            pre_pre_pre_suc_suc_id_list += lmap.lanelets[id_]['predecessor'].copy()
        
        for id_list in [pre_suc_suc_id_list, pre_pre_suc_suc_id_list, pre_pre_pre_suc_suc_id_list]:
            for id_ in id_list:
                roi_ids_waypoints.append((id_, lmap.lanelets[id_]['waypoints'])) # precision: 1
        
        return roi_ids_waypoints
    
    def roundabout_safe_to_go(self):
        safe_to_go = True
        obs_link = ''
        for obs in self.object_list.poses:
            # Roundabout ROI
            for id_, pts in self.round_roi_ids_waypoints:
                for pt in pts:
                    dis = ((obs.position.x - pt[0])**2 + (obs.position.y - pt[1])**2)**0.5
                    if dis < 1:
                        safe_to_go = False
                        tmp = id_+" "
                        obs_link += tmp
                        break
        
        return safe_to_go
    # ------------------------------------------


    def is_roi_safe_to_go(self, CS): # about to be removed
        self.roi_safe_to_go = True
        obs_link = ''
        min_lane_dis = 999
        str1 = ''
        str2 = ''
        for obs in self.object_list.poses:
            # Merge ROI
            for id_, pts in self.merge_roi_ids_waypoints:
                for pt in pts:
                    dis = ((obs.position.x - pt[0])**2 + (obs.position.y - pt[1])**2)**0.5
                    if dis < 1:
                        self.roi_safe_to_go = False
                        tmp = id_+" "
                        obs_link += tmp
                        break
                # 내 차량이 진입 시도한 이후에는 우선권이 내 차량에 있음
                for pt in pts:
                    lane_dis = ((pt[0] - CS.position.x)**2 + (pt[1] - CS.position.y)**2)**0.5
                    min_lane_dis = min(lane_dis, min_lane_dis)
                    str1 = "- Min dis to roi lane(for merge): {min_lane_dis}\n"
                    if lane_dis < 3:
                        str2 = "- Min dis < 3, ego go first\n"
                        self.roi_safe_to_go = True
                        break

            # Intersection ROI
            for id_, pts in self.inter_roi_ids_waypoints:
                for pt in pts:
                    dis = ((obs.position.x - pt[0])**2 + (obs.position.y - pt[1])**2)**0.5
                    if dis < 1:
                        self.roi_safe_to_go = False
                        tmp = id_+" "
                        obs_link += tmp
                        break
                    
            # Roundabout ROI
            for id_, pts in self.round_roi_ids_waypoints:
                for pt in pts:
                    dis = ((obs.position.x - pt[0])**2 + (obs.position.y - pt[1])**2)**0.5
                    if dis < 1:
                        self.roi_safe_to_go = False
                        tmp = id_+" "
                        obs_link += tmp
                        break
                    
        print(f"====SAFE2GO====\n- Obs on roi links: {obs_link}\n"+str1+str2+f"- Safe to go: {self.roi_safe_to_go}"+"\n")

    # curvature module utils
    def compute_curvature_radius(self, path, tg_idx=15, max_radii=90, min_radii=36): # 이 인자를 건드려서 분산도, 곡률에 민감도 결정
        curvature_radii = []
        path_len = len(path)
        
        for i in range(path_len):
            dynamic_idx = min(i, tg_idx, path_len - i - 1)

            x1, y1 = path[i - dynamic_idx]
            x2, y2 = path[i]
            x3, y3 = path[i + dynamic_idx]
            
            dx1 = x2 - x1
            dy1 = y2 - y1
            dx2 = x3 - x2
            dy2 = y3 - y2
            
            ddx = dx2 - dx1
            ddy = dy2 - dy1
            
            numerator = abs(dx1 * ddy - dy1 * ddx)
            denominator = (dx1**2 + dy1**2)**1.5
            
            if denominator != 0:
                curvature = numerator / denominator
            else:
                curvature = 1e-3  # 직선 구간에서 곡률은 0
            
            radius = 1/curvature
        
            curvature_radii.append(min(radius, max_radii))
            
            min_radii = min(min_radii, radius)
        
        processed_radii = self.postprocess_radii(curvature_radii, max_radii, min_radii)
        
        return processed_radii

    def postprocess_radii(self, raddis, max_radii, min_radii, min_target=10, max_target=20): # 이 인자를 건드려서 최저/최고속도 결정
        processed_radiis = []
        factor = (max_radii - min_radii) + 1e-3
        for el in raddis:
            val = min_target + (max_target - min_target) * (el-min_radii)/factor
            processed_radiis.append(round(val, 2)/3.6) # kph -> mps
        
        return processed_radiis
    # ------------------------------------------
    
    
    # velocity planner modules
    def CROSSWALK_module(self, CS):
        target_v_CW = 100
        str1 = ''
        str2 = ''
        if self.crosswalk_polygon is not None:
            shapely_ego = sh.Point((CS.position.x, CS.position.y))
            ego_distance_to_crosswalk = shapely_ego.distance(self.crosswalk_polygon)
            str1 = f"- Distance to crosswalk: {ego_distance_to_crosswalk:.2f}\n"
            
            for obs in self.object_list.poses:
                shapely_obj = sh.Point((obs.position.x, obs.position.y))
                obj_distance_to_crosswalk = shapely_obj.distance(self.crosswalk_polygon)
                if obj_distance_to_crosswalk < 1:
                    if 0 < ego_distance_to_crosswalk < max(CS.vEgo*3.6-15, 11):
                        target_v_CW = 10/3.6/21*(self.distance_to_stopline - 11)
                        str2 = "- Obstacle on crosswalk: close\n"
                        break
                    else:
                        str2 = "- Obstacle on crosswalk: far\n"
                else:
                    str2 = "- No obstacle\n"
        else:
            str1 = "- Nothing to do with crosswalk module\n"
            
        print(f"===CROSSWALK===\n{str1}{str2}")
            
        return target_v_CW
                
    def STOPLINE_module_morai(self, CS): # traffic_light, roundabout, intersection의 stopline 커버
        str1, str2, str3, str4, str5, str6, str7 = "", "", "", "", "", "", ""
        if self.trafficLight_type is None:
            if not self.stopline_stopped:
                if 0 < self.distance_to_stopline < max(CS.vEgo*3.6-15, 11):
                    target_v_SL = 10/3.6/21*(self.distance_to_stopline - 11)
                    if CS.vEgo > 0.02: # from Ego_topic vel.x
                        str1 = "- Run status: stopping at stopline\n"
                    else:
                        if self.stopline_timer_flag:
                            self.stopline_no_traffic_light_timer = rospy.Time.now()
                            self.stopline_timer_flag = False
                        timer, remaining_time = self.stopline_timer(3.0)
                        if timer: 
                            str2 = f"- Run status: running...({remaining_time:.2f} secs remain)\n"
                            self.stopline_stopped = True
                else:
                    target_v_SL = 100
                
            else:
                # if self.roi_safe_to_go:
                if self.intersection_safe_to_go() and self.roundabout_safe_to_go():
                    target_v_SL = 100
                    str3 = f"- Obs status: safe, go at {target_v_SL:.2f}m/s\n"
                else:
                    target_v_SL = 0
                    str4 = f"- Obs status: not safe, wait at {target_v_SL:.2f}m/s\n"

        elif self.trafficLight_type not in [48, 20, 16]: #직좌, 직황, 직
            if 0 < self.distance_to_stopline < max(CS.vEgo*3.6-15, 11):
                target_v_SL = 0
            else:
                target_v_SL = 100

        else:
            str6 = "- Nothing to do with stopline module\n"
            target_v_SL = 100

        # 정지선 위치 바뀌면 (신호등 없는 정지선) local variable 초기화
        if self.stopline_point1[0] != self.stopline_point1_last[0]:
            self.stopline_timer_flag = True
            self.stopline_stopped = False
            self.stopline_point1_last[0] = self.stopline_point1[0]
            str5 = "- Stopline with no traffic light initialized\n"
        
        str7 = f"- Target v: {target_v_SL}"
        
        print(f"====STOPLINE===\n"+f"- TrafficLight: {self.trafficLight_type}\n- Stopline s: {self.distance_to_stopline:.2f}\n"+str1+str2+str3+str4+str5+str6+str7+'\n')
        
        return target_v_SL
                
    def STOPLINE_module(self, CS): # TODO: traffic_light stopline 커버 
        str1, str2, str3, str4, str5, str6, str7 = "", "", "", "", "", "", ""
        
        if self.trafficLight_type not in [48, 20, 16]: #직좌, 직황, 직
            str6 = "- STOP signal\n"
            if 0 < self.distance_to_stopline < max(CS.vEgo*3.6-15, 11):
                target_v_SL = 0
            else:
                target_v_SL = 100

        else:
            str6 = "- GO signal\n"
            target_v_SL = 100

        # 정지선 위치 바뀌면 (신호등 없는 정지선) local variable 초기화
        if self.stopline_point1 is not None and self.stopline_point1_last is not None:
            if self.stopline_point1[0] != self.stopline_point1_last[0]:
                self.stopline_timer_flag = True
                self.stopline_stopped = False
                self.stopline_point1_last[0] = self.stopline_point1[0]
                str5 = "- Stopline with no traffic light initialized\n"
        
        str7 = f"- Target v: {target_v_SL}"
        
        print(f"====STOPLINE===\n"+f"- TrafficLight: {self.trafficLight_type}\n- Stopline s: {self.distance_to_stopline:.2f}\n"+str1+str2+str3+str4+str5+str6+str7+'\n')
        
        return target_v_SL

    def ACC_module_v1(self, CS, local_path):
        safety_distance = max(CS.vEgo*3.6-15, 9) # safe_distance
        margin = 11
        margined_safety_distance = safety_distance + margin

        nearest_s = 999
        obs_dis = 999
        obs_vel = 999
        local_idx = calc_idx(local_path, (CS.position.x, CS.position.y))

        # For ioniq
        if self.object_list.poses:
            nearest_s = self.object_list.poses[0].position.y
            obs_vel = CS.vEgo + self.object_list.poses[0].orientation.w
    
        # For morai
        # for i in range(len(local_path[local_idx:])):
        #     for obs in self.object_list.poses:
        #         obs_lane_dis = ((obs.position.x - local_path[local_idx:][i][0])**2 + (obs.position.y - local_path[local_idx:][i][1])**2)**0.5
        #         obs_dis = ((obs.position.x - CS.position.x)**2 + (obs.position.y - CS.position.y)**2)**0.5
        #         obs_vel = obs.orientation.w/3.6
        #         if obs_lane_dis < 1.75: # half of lane width
        #             if obs_dis < nearest_s:
        #                 nearest_s = obs_dis
        #             break
        
        if nearest_s <= margined_safety_distance*1.0:
            status = "danger_zone"
        elif margined_safety_distance*1.0 <= nearest_s <= margined_safety_distance*1.4:
            status = "safe_zone"
        elif margined_safety_distance*1.4 <= nearest_s:
            status = "far_zone"
        else:
            status = "zone_error"
        
        
        # s_ratio = nearest_s / margined_safety_distance
        target_v_ACC = -1.0
        if status == "danger_zone":
            target_v_ACC = (10 / 3.6 / 21) * (nearest_s - 9)
            if nearest_s < margined_safety_distance and target_v_ACC < 2*KPH_TO_MPS:
                target_v_ACC = -1
        elif status == "safe_zone":
            ratio = (nearest_s - margined_safety_distance) / (0.4 * margined_safety_distance)
            acc_safe = obs_vel * (nearest_s / margined_safety_distance)
            acc_danger = (10 / 3.6 / 21) * (nearest_s - 9)
            target_v_ACC = ratio * acc_safe + (1 - ratio) * acc_danger
        elif status == "far_zone":
            target_v_ACC = obs_vel * (nearest_s / margined_safety_distance + 1.0)
            target_v_ACC = 999
        else:
            print("[ACC] ERROR ON STATUS DECISION")


            
        str1 = f"- Status: {status}\n"
        str2 = f"- Current s: {nearest_s:.2f}\n"
        str3 = f"- Target s: {margined_safety_distance:.2f}\n"
        str4 = f"- Target_v: {target_v_ACC:.2f}\n"
        str5 = f"- nearest s: {nearest_s:.2f}\n"
        str6 = f"- obs velocity: {obs_vel:.2f}\n"
        if target_v_ACC == 999:
            str5 = f"- Nothing to do with ACC module\n"
        
        print(f"======ACC======\n{str1}{str2}{str3}{str4}{str5}{str6}")
        
        # Publish datas for plot
        # acc_plot_msg = Pose()
        # acc_plot_msg.orientation.x = margined_safety_distance
        # acc_plot_msg.orientation.y = nearest_s
        # acc_plot_msg.orientation.z = self.target_v
        # acc_plot_msg.orientation.w = CS.vEgo
        # self.pub_acc_plot.publish(acc_plot_msg)
        
        return target_v_ACC

    def ACC_module_v2(self, CS, local_path):

        s_obs = 999
        v_obs = 999
        v_ego = CS.vEgo*KPH_TO_MPS
        

        if self.object_list.poses:
            s_obs = self.object_list.poses[0].position.y
            v_obs = CS.vEgo + self.object_list.poses[0].orientation.w
        #     obs_position = (TODO)

        # idx_ego = calc_idx(local_path, (CS.position.x, CS.position.y))
        # idx_obs = calc_idx(local_path, (obs position))

        # target v
        s0 = max(CS.vEgo*3.6-15, 9) # min distance
        T_gap = 1.5
        s_ref = s0 + T_gap * v_ego
        
        gain_v_ref = 0.3
        v_ref = v_obs + gain_v_ref * (s_obs - s_ref)

        # ttc based emergency
        ttc_brake = 1.0
        ttc_warn = 2.0
        v_rel = v_obs - v_ego
        if v_rel < 0:
            ttc = s_obs / abs(v_rel)
            if ttc < ttc_brake:
                v_ref = 0.0 
            elif ttc < ttc_warn:
                v_ref = min(v_ref, v_ego)

        target_v_ACC = v_ref

        return target_v_ACC

        
    def CURVATURE_module(self, CS, local_path):
        local_point = KDTree(local_path)
        local_idx = local_point.query((CS.position.x, CS.position.y), 1)[1]
        target_v_CV = self.compute_curvature_radius(local_path)[min(len(local_path)-1, local_idx+int(CS.vEgo))] # idx: 1s after
        print(f"=====CURVE=====\n- Target v: {target_v_CV:.2f}\n")
        
        return target_v_CV
    
    def MERGE_module(self, CS):
        target_v_MG = 999
        str1 = "" 
        if self.now_scenario == "MERGE":
            # if self.roi_safe_to_go:
            if self.merge_safe_to_go(CS):
                target_v_MG = 100
                str1 = f"- Obs status: safe, go at {target_v_MG:.2f}m/s\n"
            else:
                target_v_MG = 0
                str1 = f"- Obs status: not safe, wait at {target_v_MG:.2f}m/s\n"
        else:
            str1 = "- Nothing to do with merge module"
        
        print(f"=====MERGE=====\n{str1}\n")
        
        return target_v_MG
    # ------------------------------------------
            
    def run(self, sm, lmap, tmap, my_lane_id, g_ids, pp=0, l_path=None):
        CS = sm.CS
        lgp = 0
        
        # print("CS position: ", (CS.position.x, CS.position.y))
        # print("transformed: ", self.transformed_ego_pos)
        print("--------------------------------")
        self.pub_target_v.publish(Float32(self.target_v))
        self.pub_accerror.publish(Float32(self.follow_error))
        if l_path is not None and g_ids is not None:
            local_path = l_path.copy()
            global_ids = g_ids.copy
            # if CS.cruiseState == 1:
            scenario = "integrated"
            if scenario == "integrated":
                # set scenario and roi
                # self.set_scenario_and_roi(lmap, my_lane_id, global_ids)
                
                # set safe to go
                # self.is_roi_safe_to_go(CS)
                
                # get traffic_light type
                # self.traffic_light_postprocess()
                
                # get distance to stopline
                # self.set_distance_to_stopline(CS)
                
                # get target_v
                target_v_list = []
                # target_v_list.append(self.CROSSWALK_module(CS)) 
                # target_v_list.append(self.STOPLINE_module_morai(CS))
                # target_v_list.append(self.STOPLINE_module(CS))
                # target_v_list.append(self.MERGE_module(CS))
                # target_v_list.append(self.ACC_module_v1(CS, local_path))
                target_v_list.append(self.ACC_module_v2(CS, local_path))
                # target_v_list.append(self.CURVATURE_module(CS, local_path))
                try:
                    # self.target_v = min(target_v_list)
                    # # for control test - 0612 jm
                    self.target_v = 40 * KPH_TO_MPS
                except:
                    print(target_v_list)
                    print("Error on long_planner: target_v")
                # print(f"###############\n- Current v: {CS.vEgo:.2f}\n- Target v: {self.target_v:.2f}\n\n")
                

            else:
                self.target_v = CS.vEgo

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.01:
                    lgp = 2
            else:
                lgp = 1

        return lgp
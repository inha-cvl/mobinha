

import rospy
import time
from scipy.spatial import KDTree
from std_msgs.msg import Int8, Float32, Float32MultiArray, Int8MultiArray, String, Int16MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Point32, Polygon
from visualization_msgs.msg import Marker
from morai_msgs.msg import ObjectStatusList


from selfdrive.planning.libs.map import LaneletMap, TileMap
from selfdrive.planning.libs.micro_lanelet_graph import MicroLaneletGraph
from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.rviz_utils import *
import tf2_ros

class PathPlanner:
    def __init__(self, CP):

        ## Common
        self.current_blinker_state = (0, None)
        self.state = 'WAITING'
        self.lmap = LaneletMap(CP.mapParam.path)
        self.tmap = TileMap(self.lmap.lanelets, CP.mapParam.tileSize)
        self.graph = MicroLaneletGraph(self.lmap, CP.mapParam.cutDist).graph
        self.precision = CP.mapParam.precision
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision

        ## "WAITING"
        self.get_goal = False # rviz나 GUI에서 goal 받으면 True, 유지하다가 오류 발생 시 False + "WAITING" 단계로 천이

        ## "READY"
        self.prev_head_lane_id = None
        self.now_head_lane_id = None
        self.next_head_lane_id = None
        self.global_path = None
        self.global_ids = None
        self.non_intp_path = None
        self.non_intp_id = None
        self.head_lane_ids = None
        self.global_head_ids = []

        
        ## "MOVE"
        self.windowed_global_idx = 0
        self.ego_head_id = None
        self.local_path = None
        self.local_idx = 0
        self.l_cut = 600
        self.l_nitt = 250
        self.l_tail = 50
        self.erase_global_path = []
        self.erase_global_id = []
        self.erase_global_yaw = []
        self.erase_global_k = []

        
        self.erase_global_point = None
        self.local_id = None
        self.local_path_theta = None
        self.prev_yaw = None


        self.last_s = 99999
        self.blinker = 0
        self.blinker_target_id = None
        self.renewal_path_in_progress = False
        self.renewal_path_timer = 0
        self.turnsignal = 0
        self.turnsignal_state = False
        self.renewal_path_cnt = 0
        self.lanechange_target_id = None
        # self.change_lane_flag = False

        self.lidar_obstacle = []
        self.lidar_bsd = [0, 0]
        self.around_obstacle = []
        self.look_a_head_pos = [0, 0]
        self.obstacle_detect_timer = 0
        self.nearest_obstacle_distance = -1


        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_lanelet_map = rospy.Publisher('/mobinha/planning/lanelet_map', MarkerArray, queue_size=1, latch=True)
        self.pub_goal_viz = rospy.Publisher('/mobinha/planning/goal_viz', Marker, queue_size=1, latch=True)
        self.pub_global_path = rospy.Publisher('/mobinha/global_path', Marker, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher('/mobinha/planning/local_path', Marker, queue_size=1)
        self.pub_blinkiker = rospy.Publisher('/mobinha/planning/blinker', Int8, queue_size=2)
        self.pub_goal_object = rospy.Publisher('/mobinha/planning/goal_information', Pose, queue_size=1)
        self.pub_forward_path = rospy.Publisher('/mobinha/planning/forward_path', Marker, queue_size=1)
        self.pub_lane_information = rospy.Publisher('/mobinha/planning/lane_information', Pose, queue_size=1)
        self.pub_stopline_pos = rospy.Publisher('/mobinha/planning/stopline_pos', PoseArray, queue_size=1)
        self.pub_crosswalk_pos = rospy.Publisher('/mobinha/planning/crosswalk_pos', Polygon, queue_size=1)
        
        self.pub_trajectory = rospy.Publisher('/mobinha/planning/trajectory', PoseArray, queue_size=1)
        self.pub_lidar_bsd = rospy.Publisher('/mobinha/planning/lidar_bsd', Point, queue_size=1)
        self.pub_local_path_theta = rospy.Publisher('/mobinha/planning/local_path_theta', Float32MultiArray, queue_size=1)
        self.pub_local_path_radius = rospy.Publisher('/mobinha/planning/local_path_radius', Float32MultiArray, queue_size=1)
        self.pub_local_path_k = rospy.Publisher('/mobinha/planning/local_path_k', Float32MultiArray, queue_size=1)
        self.prevRoadPolygon_pub = rospy.Publisher('/prevRoadPolygon', Marker, queue_size=10)
        self.nowRoadPolygon_pub = rospy.Publisher('/nowRoadPolygon', Marker, queue_size=10)
        self.nextRoadPolygon_pub = rospy.Publisher('/nextRoadPolygon', Marker, queue_size=10)
        self.crosswalkPolygon_pub = rospy.Publisher('/crosswalkPolygon', Marker, queue_size=10)
        self.stoplinePolygon_pub = rospy.Publisher('/stoplinePolygon', Marker, queue_size=10)
        self.crosswalkPolygon_pub = rospy.Publisher('/crosswalkPolygon', MarkerArray, queue_size=10)
        
        self.pub_right_turn_situation = rospy.Publisher('/mobinha/planning/right_turn_situation_real', Int8MultiArray, queue_size=1)
        # self.schoolzone_state_pub = rospy.Publisher('/mobinha/planning/schoolzone', Int16MultiArray, queue_size=5)
        # self.schoolzone_polygon_pub = rospy.Publisher('/schoolzone_polygon', MarkerArray, queue_size=10)
        # self.schoolzone_state_pub = rospy.Publisher('/schoolzone', Float32MultiArray, queue_size=5)
        
        self.map_name = rospy.get_param('map_name', 'None')
        if self.map_name == 'songdo':
            lanelet_map_viz = VectorMapVis(self.lmap.map_data)
        else:
            lanelet_map_viz = LaneletMapViz(self.lmap.lanelets, self.lmap.for_viz)

        self.pub_lanelet_map.publish(lanelet_map_viz)

        rospy.Subscriber('/move_base_simple/single_goal', PoseStamped, self.single_goal_cb) # rviz 2-D nav goal
        rospy.Subscriber('/mobinha/visualize/scenario_goal',PoseArray, self.scenario_goal_cb) # scenario button from GUI
        rospy.Subscriber('/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber('/mobinha/perception/nearest_obstacle_distance', Float32, self.nearest_obstacle_distance_cb)
        rospy.Subscriber('/mobinha/control/look_ahead', Marker, self.look_a_head_cb)
        rospy.Subscriber('/mobinha/perception/around_obstacle', PoseArray, self.around_obstacle_cb)
        rospy.Subscriber('/turnsignal', Int8, self.blinker_cb)
        self.goal_pts = []

    def blinker_cb(self, msg):
        self.turnsignal = msg.data

    def single_goal_cb(self, msg):
        self.goal_pts = [(msg.pose.position.x, msg.pose.position.y)]
        self.get_goal = True

    def scenario_goal_cb(self, msg): # when scenario button clicked
        scenario_goal = []
        for pose in msg.poses:
            scenario_goal.append((pose.position.x, pose.position.y))
        if len(self.goal_pts) != 0:
            if self.goal_pts != scenario_goal:
                self.goal_pts = scenario_goal
                self.get_goal = True
                self.state = 'READY'
        else:
            self.goal_pts = scenario_goal
            self.get_goal = True
        
        # self.goal_pts = scenario_goal
        # self.get_goal = True

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.z)for pose in msg.poses]

    def nearest_obstacle_distance_cb(self, msg):
        self.nearest_obstacle_distance = round(msg.data, 5)  # nearest obstacle
    
    def around_obstacle_cb(self, msg):
        # idx, s, d, enu_x, enu_y, v, track id
        self.around_obstacle = [(pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.w, pose.orientation.z)for pose in msg.poses]
    
    def look_a_head_cb(self, msg):
        self.look_a_head_pos = [msg.pose.position.x, msg.pose.position.y]

    def returnAppendedNonIntpPath(self, ego_pos):
        appended_non_intp_path = []
        appended_non_intp_id = []
        appended_head_lane_ids = []
        goal_pt = None
        temp_pt = None

        for pt in self.goal_pts:
            goal_pt = pt
            shortest_path = []

            g_id = None
            g_idx = None
            
            if temp_pt is None:
                temp_pt = ego_pos
            ego_lanelets = lanelet_matching(
                self.tmap.tiles, self.tmap.tile_size, temp_pt)
            if ego_lanelets is not None:
                e_id, e_idx = ego_lanelets
            else:
                rospy.logerr('Failed to match [ego] to lanelets, Insert Goal Again')
                self.get_goal = False
                self.state = 'WAITING'
                return None, None, None
            
            goal_lanelets = lanelet_matching(self.tmap.tiles, self.tmap.tile_size, goal_pt)
            if goal_lanelets is not None:
                g_id, g_idx = goal_lanelets
            else:
                rospy.logerr('Failed to match [goal] to lanelets, Insert Goal Again')
                self.get_goal = False
                self.state = 'WAITING'
                return None, None, None

            # 다익스트라에 사용할 정보로 변환 (waypoint -> node)
            e_node = node_matching(self.lmap.lanelets, e_id, e_idx)
            g_node = node_matching(self.lmap.lanelets, g_id, g_idx)


            if e_node == g_node:
                shortest_path = ([e_node], 0) # (path, cost)
            else:
                shortest_path = optimized_dijkstra(self.graph, e_node, g_node)
            if shortest_path is not None:
                shortest_path = shortest_path[0]
            else:
                rospy.logerr('Failed to match ego to lanelets, Insert Goal Again')
                self.get_goal = False
                self.state = 'WAITING'
                return None, None, None
        

            # 주행에 사용할 정보로 변환 (node -> waypoint)

            # shortest path = ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0']
            non_intp_path, non_intp_id = node_to_waypoints2(self.lmap.lanelets, shortest_path)
            # non_intp_path = [[-248.78557822582127, -283.93591207290297], [-238.73866953211646, -272.1329630255155], [-229.9878153272595, -261.85324776952456], [-221.28433089503935, -251.63028019257519], [-213.18159024154102, -242.11210912113202], [-205.30837435857626, -232.8639387157932]]
            # non_intp_id = ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0']
            # 기본 형식은 'xxx_x' 리스트 형식인데, 'xxx'형식이 들어오면 node_to_waypoints2에서 'xxx_x'로 형식 맞춰줌

            # 슬라이싱해서 앞/뒤 중복 제거 (다익스트라 결과에서 앞뒤 여유 포함, 시작점/끝점이 대표점이랑 정확히 일치하지 않을 경우)
            intp_start_idx = calc_idx(non_intp_path, temp_pt) 
            # temp_pt 초기값은 차량 위치, 이후부턴 직전 goal_pt
            intp_last_idx = calc_idx(non_intp_path, goal_pt)
            # start idx = 0
            # end idx = 5
            non_intp_path = non_intp_path[intp_start_idx:intp_last_idx+1]
            non_intp_id = non_intp_id[intp_start_idx:intp_last_idx+1]
            # non intp id = ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0'] 

            appended_non_intp_path.extend(non_intp_path)
            appended_non_intp_id.extend(non_intp_id)
            appended_head_lane_ids.extend(shortest_path)

            temp_pt = goal_pt
        
        # id 앞부분만 취하고 중복제거
        appended_head_lane_ids = set_lane_ids(appended_head_lane_ids)
        # appended_head_lane_ids = ['865', '866', '781'] 
    
        

        return appended_non_intp_path, appended_non_intp_id, appended_head_lane_ids

    def delete_node_for_smooth_path(self, non_intp_path, non_intp_id):
        before_n = non_intp_id[0].split('_')[0]
        for i, n in enumerate(non_intp_id):
            splited_id = n.split('_')[0]
            if splited_id != before_n :
                my_neighbor_id = get_my_neighbor(self.lmap.lanelets, before_n)
                if not compare_id(splited_id, my_neighbor_id):
                    del non_intp_path[i]
                    del non_intp_id[i]
                before_n = splited_id
                    
    def run(self, sm):
        CS = sm.CS
        pp = 0

        if self.state == 'WAITING':
            time.sleep(1)
            if self.get_goal == True: # goal 받으면
                self.state = 'READY' 
            pp = 3

        elif self.state == 'READY':  
            non_intp_path = None
            non_intp_id = None
            head_lane_ids = None

            start = time.time()
            non_intp_path, non_intp_id, head_lane_ids = self.returnAppendedNonIntpPath([CS.position.x, CS.position.y])
            # (x,y) 경로, 경로점에 대응하는 id, 대표 id(중복제거)
            # ex) [(x0, y0), ...], ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0'], ['865', '866', '781']  
            print(f"<Elapsed:Scenario3> AppendedNonIntpPath: {(time.time() - start):.3f}")

            # Error check
            if non_intp_path is None or non_intp_id is None:
                rospy.logerr('An error occurred, unable to process path. Returning to WAITING state.')
                self.get_goal = False
                self.state = 'WAITING'
                pp = 3
                return pp, None
            
            # 중복 제거
            start = time.time()
            self.delete_node_for_smooth_path(non_intp_path, non_intp_id)
            print(f"<Elapsed:Scenario3> delete_node...: {(time.time() - start):.3f}")
            
            start = time.time()
            global_path, self.last_s = ref_interpolate_2d(non_intp_path, self.precision)
            print(f"<Elapsed:Scenario3> ref_interpolate_2d: {(time.time() - start):.3f}")

            start = time.time()
            global_path, global_yaw, global_k = smooth_compute_yaw_and_curvature(global_path, self.precision)
            print(f"<Elapsed:Scenario3> smooth_compute_yaw...: {(time.time() - start):.3f}")

            start = time.time()
            global_ids = id_interpolate(non_intp_path, global_path, non_intp_id)
            # global ids는 고해상도 path를 이루는 모든 점의 id를 저장한 list
            # .., '1069_4', '1069_4', '1069_4', '1069_4', '1069_4', '1069_4', '1069_4', '1069_4', '1069_4']
            print(f"<Elapsed:Scenario3> id_interpolate: {(time.time() - start):.3f}")
            
            # 변수 저장
            self.global_path = global_path
            self.global_ids = global_ids
            self.non_intp_path = non_intp_path
            self.non_intp_id = non_intp_id
            self.head_lane_ids = head_lane_ids

            # Global Path의 Head-ids
            for id in global_ids:
                val = id.split("_")[0]
                if val not in self.global_head_ids:
                    self.global_head_ids.append(val)

            # Lane Window 초기설정
            if len(head_lane_ids) >= 2:
                self.prev_head_lane_id = None
                self.now_head_lane_id = head_lane_ids[0]
                self.next_head_lane_id = head_lane_ids[1]
            elif len(head_lane_ids) == 1:
                self.prev_head_lane_id = None
                self.now_head_lane_id = head_lane_ids[0]
                self.next_head_lane_id = head_lane_ids[0]
            else:
                self.prev_head_lane_id = None
                self.now_head_lane_id = None
                self.next_head_lane_id = None

            # 잔여 global path 정보 update에 사용될 변수 저장
            self.global_path = global_path.tolist()
            self.global_id = global_ids
            self.global_yaw = global_yaw.tolist()
            self.global_k = global_k.tolist()

            # Viz: Global Path
            global_path_viz = FinalPathViz(self.global_path)
            self.pub_global_path.publish(global_path_viz)

            # Viz: Goal
            goal_viz = GoalViz(self.goal_pts[-1])
            self.pub_goal_viz.publish(goal_viz)

            # state 전환
            self.state = 'MOVE'

            pp = 0

        elif self.state == 'MOVE':
            global_point = KDTree(self.global_path)
            self.global_idx = global_point.query((CS.position.x, CS.position.y), 1)[1]

            self.global_idx = calc_idx(self.global_path, (CS.position.x, CS.position.y))
            if abs(self.global_idx-self.windowed_global_idx) <= 50:
                self.windowed_global_idx = self.global_idx

            s = self.windowed_global_idx * self.precision  # 진행 거리, m

            self.ego_head_id = self.global_ids[self.global_idx].split('_')[0]
            # self.ego_head_id: 현재 속한 id의 head

            # Lane Window Update
            if self.ego_head_id == self.next_head_lane_id: # next head lane에 진입하면
                if len(self.head_lane_ids) < 2: # 끝에 다다랐을 경우
                    self.prev_head_lane_id = self.now_head_lane_id
                    self.now_head_lane_id = self.next_head_lane_id
                else: # 일반적인 상황일 때 한칸씩 앞으로 shift
                    self.prev_head_lane_id = self.now_head_lane_id
                    self.now_head_lane_id = self.next_head_lane_id
                    self.next_head_lane_id = self.head_lane_ids[1]
                    self.head_lane_ids = self.head_lane_ids[1:]
            # print("Prev lane id", self.prev_head_lane_id) # heesang
            # print("Curr lane id", self.now_head_lane_id)
            # print("Next lane id", self.next_head_lane_id)
            if self.local_path is None or self.local_idx > 350:

                start_idx = max(self.global_idx - 50, 0)
                end_idx   = min(self.global_idx + 600, len(self.global_path))

                self.local_path = self.global_path[start_idx:end_idx]
                self.local_id   = self.global_ids[start_idx:end_idx]
                self.local_yaw  = self.global_yaw[start_idx:end_idx]
                self.local_k    = self.global_k[start_idx:end_idx]

                self.pub_local_path_theta.publish(Float32MultiArray(data=self.local_yaw))
                self.pub_local_path_k.publish(Float32MultiArray(data=self.local_k))

            if self.local_path is not None:
                local_point = KDTree(self.local_path)
                self.local_idx = local_point.query((CS.position.x, CS.position.y), 1)[1]
                # print("local idx is: ", self.local_idx)
                # print("global idx is: ", self.global_idx)

                splited_local_id = (self.local_id[self.local_idx]).split('_')[0]
                my_neighbor_id = get_my_neighbor(self.lmap.lanelets, splited_local_id) 
                forward_direction = get_forward_direction(self.lmap.lanelets, self.now_head_lane_id, self.head_lane_ids)
                stopline_idx, stopline_wps = get_nearest_stopline(self.lmap.lanelets, self.lmap.stoplines, self.now_head_lane_id, self.head_lane_ids, local_point)

                # ## Lane Change Local Signal Ver.
                # if self.turnsignal != 0 and not self.turnsignal_state:
                #     renew_path, renew_ids = get_lane_change_path(self.local_id, self.turnsignal, self.local_idx, self.lmap.lanelets, 
                #                                                  self.local_path[self.local_idx+120:self.local_idx+240])
                #     if renew_path != None:
                #         for i, renew_pt in enumerate(renew_path):
                #             self.local_path[self.local_idx+120+i]=renew_pt
                #             self.local_id[self.local_idx+120+i]=renew_ids[i]
                #         if  self.local_idx+240+30+10 < len(self.local_path)+1:
                #             force_interpolate_path, _ = ref_interpolate_2d([self.local_path[self.local_idx+120-30], self.local_path[self.local_idx+120+30]], self.precision)
                #             for i, force_pt in enumerate(force_interpolate_path):
                #                 self.local_path[self.local_idx+120-30+i]=force_pt                  
                #             force_interpolate_path, _ = ref_interpolate_2d([self.local_path[self.local_idx+240-30], self.local_path[self.local_idx+240+30]], self.precision)
                #             for i, force_pt in enumerate(force_interpolate_path):
                #                 self.local_path[self.local_idx+240-30+i]=force_pt
                #         else:
                #             print("remaining local pass is too short.")
                #             pass
                #     else:
                #         print("The link to change lanes does not exist.")
                #         pass
                #     self.turnsignal_state = True
                # elif self.turnsignal == 0:
                #     self.turnsignal_state = False
                
                ## Blinker
                blinker, target_id = get_blinker_and_targetid(self.local_idx, self.lmap.lanelets, self.local_id, my_neighbor_id, CS.vEgo, self.M_TO_IDX, splited_local_id) 
                                                                        #,splited_local_id, self.lanechange_target_id, self.change_lane_flag)

                if blinker != 0 and self.blinker_target_id == None:
                    self.blinker_target_id = target_id
                    self.blinker = blinker
                elif splited_local_id == self.blinker_target_id:
                    self.blinker_target_id = None
                    self.blinker = 0

                if target_id != None:
                    self.lanechange_target_id = target_id
                    # self.change_lane_flag = True
                if self.lanechange_target_id == splited_local_id:
                    # self.change_lane_flag = False
                    self.renewal_path_cnt = 0

                # forward_curvature, rot_x, rot_y, trajectory = get_forward_curvature(self.local_idx, self.local_path, CS.yawRate, CS.vEgo, 0, self.lmap.lanelets, self.now_head_lane_id, self.next_head_lane_id, self.M_TO_IDX)
                forward_curvature, rot_x, rot_y, trajectory = get_forward_curvature(self.local_idx, self.local_path, CS.yawRate, CS.vEgo, blinker, self.lmap.lanelets, self.now_head_lane_id, self.next_head_lane_id, self.M_TO_IDX)
                lane_change_point = get_lane_change_point(self.local_id, self.local_idx, my_neighbor_id)

                ## Lane Change Local Path Planning
                # d = (lane_change_point - self.local_idx)*self.IDX_TO_M
                # timetoarrivelanechangepoint = d/CS.vEgo if CS.vEgo != 0 else d*1000

                ## LIDAR BSD
                self.lidar_bsd = [0, 0]

                # if blinker != 0 and not self.renewal_path_in_progress:
                #     # look a head's idx's id == lane id => stop looking BSD
                #     look_a_head_idx = local_point.query(self.look_a_head_pos, 1)[1]
                #     look_a_head_id = self.local_id[look_a_head_idx].split('_')[0]
                #     get_look_a_head_id = compare_id(look_a_head_id, my_neighbor_id)
                #     renew_a = 30 # uniti: idx
                #     renew_b = 120 # unit : idx
                #     for obs in self.around_obstacle:
                #         print(obs)
                #         print(get_look_a_head_id)
                #         #Left
                #         if blinker == 1 and get_look_a_head_id and -4.05<obs[2]<-1.8 and lane_change_point<(len(self.local_path)-1): # frenet d coordinate left. 
                #             #TODO: if left lane change, get prev,now,next leftBound and check obstacle where is it. 
                #             _, _, _, isCarInRoad = is_car_inside_combined_road((obs[3],obs[4]),self.lmap.lanelets, self.prev_head_lane_id, self.now_head_lane_id, self.next_head_lane_id)
                #             if isCarInRoad:
                #                 vTargetCar = (obs[5] + CS.vEgo) # unit: m/s
                #                 targetcarmovingdistance = vTargetCar * timetoarrivelanechangepoint # unit: m
                #                 safedistance = vTargetCar*MPS_TO_KPH - 15 # unit: m 
                #                 print("d: ", d)
                #                 print("targetmove: ", targetcarmovingdistance)
                #                 print(safedistance)
                #                 print("obs distance:",(obs[1] - self.local_idx)*self.IDX_TO_M)
                #                 if safedistance < 10:
                #                     safedistance = 10 # 5 * 2 : front and back 
                #                 safe_space = (safedistance/2)
                #                 obs_distance = (obs[1] - self.local_idx)*self.IDX_TO_M
                #                 if targetcarmovingdistance + obs_distance - safe_space < d < targetcarmovingdistance + obs_distance + safe_space:
                #                     #get renewable local path
                #                     renew_path, renew_ids = get_renew_path(self.local_id, blinker, lane_change_point, self.lmap.lanelets, 
                #                                                         self.local_path[lane_change_point:lane_change_point+renew_b], self.local_path[lane_change_point-renew_a:lane_change_point])
                #                     self.lidar_bsd = [1, 0]
                #                     if renew_path != None:
                #                         for i, renew_pt in enumerate(renew_path):
                #                             self.local_path[lane_change_point-renew_a+i]=renew_pt
                #                             self.local_id[lane_change_point-renew_a+i]=renew_ids[i]
                #                         if  lane_change_point+renew_a+renew_b+25 < len(self.local_path)+1:
                #                             force_interpolate_path,_ = ref_interpolate([self.local_path[lane_change_point-renew_a+renew_b], self.local_path[lane_change_point+renew_a+renew_b]], self.precision)
                #                             print("left BSD")
                #                             for i, force_pt in enumerate(force_interpolate_path):
                #                                 self.local_path[lane_change_point-renew_a+renew_b+i]=force_pt
                #                             self.renewal_path_in_progress = True
                #                             self.renewal_path_cnt += 1
                #                             self.renewal_path_timer = time.time()
                #                             break # multi obstacle passing
                #                         else:
                #                             pass
                #                     else:
                #                         print("Take Over Request")
                #                         pp = 4
                #                         if pp == 4:
                #                             self.renewal_path_cnt += 1
                #                         if self.renewal_path_cnt > 30:
                #                             self.renewal_path_cnt = 0
                #                         return pp, self.local_path
                #         elif blinker == 2 and get_look_a_head_id and 1.8<obs[2]<4.05 and lane_change_point<(len(self.local_path)-1): # frenet d coordinate right.
                #             _, _, _, isCarInRoad = is_car_inside_combined_road((obs[3],obs[4]),self.lmap.lanelets, self.prev_head_lane_id, self.now_head_lane_id, self.next_head_lane_id)
                #             if isCarInRoad:
                #                 vTargetCar = (obs[5] + CS.vEgo) # unit: m/s
                #                 targetcarmovingdistance = vTargetCar * timetoarrivelanechangepoint # unit: m
                #                 safedistance = vTargetCar*MPS_TO_KPH - 15 # unit: m 
                #                 print("d: ", d)
                #                 print("targetmove: ", targetcarmovingdistance)
                #                 print(safedistance)
                #                 print("obs distance:",(obs[1] - self.local_idx)*self.IDX_TO_M)
                #                 if safedistance < 10:
                #                     safedistance = 10 # 5 * 2 : front and back 
                #                 safe_space = (safedistance/2)
                #                 obs_distance = (obs[1] - self.local_idx)*self.IDX_TO_M
                #                 if targetcarmovingdistance + obs_distance - safe_space < d < targetcarmovingdistance + obs_distance + safe_space:
                                    
                #                     #get renewable local path
                #                     renew_path, renew_ids = get_renew_path(self.local_id, blinker, lane_change_point, self.lmap.lanelets, 
                #                                                         self.local_path[lane_change_point:lane_change_point+renew_b], self.local_path[lane_change_point-renew_a:lane_change_point])
                #                     self.lidar_bsd = [0, 1]
                #                     if renew_path != None:
                #                         for i, renew_pt in enumerate(renew_path):
                #                             self.local_path[lane_change_point-renew_a+i]=renew_pt
                #                             self.local_id[lane_change_point-renew_a+i]=renew_ids[i]
                #                         if  lane_change_point+renew_a+renew_b+25 < len(self.local_path)+1:
                #                             force_interpolate_path,_ = ref_interpolate([self.local_path[lane_change_point-renew_a+renew_b], self.local_path[lane_change_point+renew_a+renew_b]], self.precision)
                #                             print("right BSD")
                #                             for i, force_pt in enumerate(force_interpolate_path):
                #                                 self.local_path[lane_change_point-renew_a+renew_b+i]=force_pt
                #                             self.renewal_path_in_progress = True
                #                             self.renewal_path_cnt += 1
                #                             self.renewal_path_timer = time.time()
                #                             break
                #                         else:
                #                             pass
                #                     else:
                #                         print("Take Over Request")
                #                         pp = 4
                #                         if pp == 4:
                #                             self.renewal_path_cnt += 1
                #                         if self.renewal_path_cnt > 30:
                #                             self.renewal_path_cnt = 0
                #                         return pp, self.local_path
                # elif self.renewal_path_cnt >= 2:
                #     print("Take Over Request(continuos 2 times)")
                #     pp = 4
                #     if pp == 4:
                #         self.renewal_path_cnt += 1
                #     if self.renewal_path_cnt > 30:
                #         self.renewal_path_cnt = 0
                #     return pp, self.local_path

                # elif time.time() - self.renewal_path_timer > 1.5:
                #     self.renewal_path_in_progress = False

                # link_idx = findMyLinkIdx(self.lmap.lanelets, splited_local_id, CS.position.x, CS.position.y)
                # lane_position = removeVegetationFromRoadside(self.lmap.lanelets, splited_local_id, link_idx)

                # # Pubulish Lane Information
                # pose = Pose()
                # pose.position.x = int(splited_local_id)
                # pose.position.y = get_direction_number(self.lmap.lanelets, splited_local_id, forward_direction)
                # pose.position.z = stopline_idx
                # pose.orientation.x = forward_curvature
                # pose.orientation.y = self.local_idx
                # pose.orientation.z = lane_change_point
                # pose.orientation.w = lane_position
                # self.pub_lane_information.publish(pose)

                
                # Publish current link's stopline position
                if len(stopline_wps)>0:
                    pose_array_msg = PoseArray()
                    
                    pose1 = Pose()
                    pose1.position.x = stopline_wps[0][0]
                    pose1.position.y = stopline_wps[0][1]
                    pose1.position.z = 0
                    
                    pose2 = Pose()
                    pose2.position.x = stopline_wps[-1][0]
                    pose2.position.y = stopline_wps[-1][1]
                    pose2.position.z = 0
                    
                    pose_array_msg.poses.append(pose1)
                    pose_array_msg.poses.append(pose2)
                    
                    self.pub_stopline_pos.publish(pose_array_msg)
                else:
                    print("[Path planner.py] No stopline detected")
                poseArray = PoseArray()
                for i, x in enumerate(rot_x):
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = rot_y[i]
                    pose.position.z = forward_curvature
                    poseArray.poses.append(pose)
                self.pub_trajectory.publish(poseArray)

                bsd = Point()
                bsd.x = self.lidar_bsd[0]
                bsd.y = self.lidar_bsd[1]
                self.pub_lidar_bsd.publish(bsd)

                forward_path_viz = ForwardPathViz(trajectory)
                self.pub_forward_path.publish(forward_path_viz)
                self.pub_blinkiker.publish(blinker)

                pose = Pose()
                pose.position.x = 1
                pose.position.y = self.last_s  # m
                pose.position.z = s
                # local_path my theta
                target_heading = estimate_theta(self.local_path, self.local_idx) * 180 / np.pi
                pose.orientation.x = target_heading
                # CTE
                pose.orientation.y = calculate_cte(self.local_path[self.local_idx], self.local_path[self.local_idx+1], (CS.position.x, CS.position.y))
                self.pub_goal_object.publish(pose)
                

                # crosswalkViz
                tree = KDTree(self.lmap.lanelets[self.ego_head_id]['waypoints'])
                cur_id_idx = tree.query((CS.position.x, CS.position.y), 1)[1]
                global_id_idx = self.global_head_ids.index(self.now_head_lane_id) #now_head_lane_id는 역행하지 않음
                
                
                remaining_global_ids = self.global_head_ids[global_id_idx:] # 완

                # crosswalk_ids_points = get_crosswalk_ids_points(self.lmap.lanelets, self.lmap.surfacemarks, remaining_global_ids, cur_id_idx) # self.head_lane_ids는 정렬 x
                # merged_crosswalk_ids_points = merge_polygons(crosswalk_ids_points)

                merged_crosswalk_ids_points = [get_crosswalk_ids_points(self.lmap.lanelets, self.lmap.surfacemarks, remaining_global_ids, cur_id_idx)] # self.head_lane_ids는 정렬 x
                # merged_crosswalk_ids_points = merge_polygons(crosswalk_ids_points)

                # crosswalkPolygonMarkers = CrosswalkViz(merged_crosswalk_ids_points)
                # self.crosswalkPolygon_pub.publish(crosswalkPolygonMarkers)
                
                crosswalk_pos = Polygon()
                merged_crosswalk_ids_points = [] ## ddambbang
                if len(merged_crosswalk_ids_points) > 0:
                    min_distance = float('inf')
                    nearest_crosswalk = merged_crosswalk_ids_points[0]
                    
                    for crosswalk in merged_crosswalk_ids_points:
                        first_point = crosswalk[1][0]
                        distance = euc_distance(first_point, (CS.position.x, CS.position.y))
                        # distance = math.sqrt((first_point[0]- CS.position.x)**2 + (first_point[1] - CS.position.y)**2)

                        if distance < min_distance:
                            min_distance = distance
                            nearest_crosswalk = crosswalk
                
                    for wp in nearest_crosswalk[1]:
                        pt = Point32()
                        pt.x = wp[0]
                        pt.y = wp[1]
                        crosswalk_pos.points.append(pt)
                
                self.pub_crosswalk_pos.publish(crosswalk_pos)

                # Local Path Viz
                local_path_viz = LocalPathViz(self.local_path)
                self.pub_local_path.publish(local_path_viz) 

                # stoplineViz
                stoplinePolygonmarker = StopLineViz(stopline_wps)
                self.stoplinePolygon_pub.publish(stoplinePolygonmarker)
                
            if self.last_s - s < 5.0:
                self.state = 'ARRIVED'
            pp = 1

        elif self.state == 'ARRIVED':
            pp = 2

        return pp, self.ego_head_id, self.global_head_ids, self.local_path, self.lmap, self.tmap



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

        self.current_blinker_state = (0, None)
        self.state = 'WAITING'
        self.lmap = LaneletMap(CP.mapParam.path)
        self.tmap = TileMap(self.lmap.lanelets, CP.mapParam.tileSize)
        self.graph = MicroLaneletGraph(self.lmap, CP.mapParam.cutDist).graph
        self.precision = CP.mapParam.precision
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision

        self.temp_pt = None
        self.global_path = None
        self.global_ids = None
        self.splited_global_ids = None
        
        self.erase_global_point = None
        self.non_intp_path = None
        self.non_intp_id = None
        self.local_path = None
        self.local_id = None
        self.temp_global_idx = 0
        self.local_path_theta = None
        self.prev_yaw = None
        # self.global_yaw, self.global_k = [], []

        self.get_goal = False # rviz나 GUI에서 goal 받으면 True, 유지하다가 오류 발생 시 False + "WAITING" 단계로 천이

        self.l_idx = 0
        self.erase_global_path = []
        self.erase_global_id = []
        self.erase_global_yaw = []
        self.erase_global_k = []
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
        self.lidar_bsd = [0,0]
        self.around_obstacle = []
        self.look_a_head_pos = [0,0]
        self.obstacle_detect_timer = 0
        self.nearest_obstacle_distance = -1

        self.splited_id = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_lanelet_map = rospy.Publisher('/mobinha/planning/lanelet_map', MarkerArray, queue_size=1, latch=True)
        self.pub_goal_viz = rospy.Publisher('/mobinha/planning/goal_viz', Marker, queue_size=1, latch=True)
        self.pub_global_path = rospy.Publisher('/mobinha/global_path', Marker, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher('/mobinha/planning/local_path', Marker, queue_size=1)
        self.pub_goal_object = rospy.Publisher('/mobinha/planning/goal_information', Pose, queue_size=1)
        self.pub_forward_path = rospy.Publisher('/mobinha/planning/forward_path', Marker, queue_size=1)
        self.pub_lane_information = rospy.Publisher('/mobinha/planning/lane_information', Pose, queue_size=1)
        self.pub_stopline_pos = rospy.Publisher('/mobinha/planning/stopline_pos', PoseArray, queue_size=1)
        self.pub_crosswalk_pos = rospy.Publisher('/mobinha/planning/crosswalk_pos', Polygon, queue_size=1)
        
        self.pub_local_id = rospy.Publisher('/mobinha/planning/local_id', String, queue_size=1)
        ### 2024.05.23 test
        self.pub_splited_local_id = rospy.Publisher('/mobinha/planning/splited_localid', String, queue_size=1)
        ### 2024.05.23 test
        ### 2024.05.24 test
        self.pub_turing_flag = rospy.Publisher('/mobinha/planning/turning_flag', String, queue_size=1)
        self.pub_turning_velocity = rospy.Publisher('/mobinha/planning/turning_target_v', Int8, queue_size=1)
        ### 2024.05.24 test
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
        self.schoolzone_state_pub = rospy.Publisher('/mobinha/planning/schoolzone', Int16MultiArray, queue_size=5)
        self.schoolzone_polygon_pub = rospy.Publisher('/schoolzone_polygon', MarkerArray, queue_size=10)
        self.schoolzone_state_pub = rospy.Publisher('/schoolzone', Float32MultiArray, queue_size=5)
        
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

    def returnAppendedNonIntpPath(self):
        appended_non_intp_path = []
        appended_non_intp_id = []
        appended_head_lane_ids = []
        goal_pt = None

        for pt in self.goal_pts:
            goal_pt = pt
            shortest_path = []

            g_id = None
            g_idx = None
            
            ego_lanelets = lanelet_matching(
                self.tmap.tiles, self.tmap.tile_size, self.temp_pt)
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


            start = time.time()
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
            intp_start_idx = calc_idx(non_intp_path, self.temp_pt) 
            # self.temp_pt 초기값은 차량 위치, 이후부턴 직전 goal_pt
            intp_last_idx = calc_idx(non_intp_path, goal_pt)
            # start idx = 0
            # end idx = 5
            non_intp_path = non_intp_path[intp_start_idx:intp_last_idx+1]
            non_intp_id = non_intp_id[intp_start_idx:intp_last_idx+1]
            # non intp id = ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0'] 

            appended_non_intp_path.extend(non_intp_path)
            appended_non_intp_id.extend(non_intp_id)
            appended_head_lane_ids.extend(shortest_path)

            self.temp_pt = goal_pt
        
        # id 앞부분만 취하고 중복제거
        appended_head_lane_ids = set_lane_ids(appended_head_lane_ids)
        # appended_head_lane_ids = ['865', '866', '781'] 
    
        goal_viz = GoalViz(goal_pt)
        self.pub_goal_viz.publish(goal_viz)

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
            if self.get_goal == True:
                self.state = 'READY'
            pp = 3

        elif self.state == 'READY':
            non_intp_path = None
            non_intp_id = None
            head_lane_ids = None
            self.local_path = None
            self.local_id = None
            self.temp_global_idx = 0
            self.l_idx = 0
            self.l_cut = 600
            self.l_nitt = 250
            self.l_tail = 50
            self.temp_pt = [CS.position.x, CS.position.y]

            start = time.time()
            non_intp_path, non_intp_id, head_lane_ids = self.returnAppendedNonIntpPath()
            # (x,y) 경로, 경로점에 대응하는 id, 대표 id(중복제거)
            # ex) [(x0, y0), ...], ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0'], ['865', '866', '781']  
            print(f"<Elapsed:Scenario3> AppendedNonIntpPath: {(time.time() - start):.3f}")

            if None in [non_intp_path, non_intp_id]:
                rospy.logerr('An error occurred, unable to process path. Returning to WAITING state.')
                self.get_goal = False
                self.state = 'WAITING'
                pp = 3
                return pp, None
            
            start = time.time()
            self.delete_node_for_smooth_path(non_intp_path, non_intp_id)
            print(f"<Elapsed:Scenario3> delete_node...: {(time.time() - start):.3f}")

            if non_intp_path is not None:
                self.state = 'MOVE'
                
                start = time.time()
                global_path, self.last_s = ref_interpolate_2d(non_intp_path, self.precision)
                print(f"<Elapsed:Scenario3> ref_interpolate_2d: {(time.time() - start):.3f}")

                start = time.time()
                global_path, global_yaw, global_k = smooth_compute_yaw_and_curvature(global_path, self.precision)
                print(f"<Elapsed:Scenario3> smooth_compute_yaw...: {(time.time() - start):.3f}")

                start = time.time()
                global_ids = id_interpolate(non_intp_path, global_path, non_intp_id)
                print(f"<Elapsed:Scenario3> id_interpolate: {(time.time() - start):.3f}")
                
                self.global_path = global_path
                self.global_ids = global_ids
                self.non_intp_path = non_intp_path
                self.non_intp_id = non_intp_id
                self.head_lane_ids = head_lane_ids

                self.splited_global_ids = []
                for id in global_ids:
                    val = id.split("_")[0]
                    if val not in self.splited_global_ids:
                        self.splited_global_ids.append(val)

                if len(head_lane_ids) >= 2:
                    self.prev_head_lane_id = None
                    self.next_head_lane_id = head_lane_ids[1]
                    self.now_head_lane_id = head_lane_ids[0]
                elif len(head_lane_ids) == 1:
                    self.prev_head_lane_id = None
                    self.now_head_lane_id = head_lane_ids[0]
                    self.next_head_lane_id = head_lane_ids[0]
                else:
                    self.prev_head_lane_id = None
                    self.next_head_lane_id = None
                    self.now_head_lane_id = None
                self.erase_global_path = global_path
                self.erase_global_id = global_ids
                self.erase_global_yaw = global_yaw
                self.erase_global_k = global_k
                global_path_viz = FinalPathViz(self.global_path)
                self.pub_global_path.publish(global_path_viz)

            pp = 0

        elif self.state == 'MOVE':

            idx = calc_idx(self.global_path, (CS.position.x, CS.position.y))

            if abs(idx-self.temp_global_idx) <= 50:
                self.temp_global_idx = idx

            s = self.temp_global_idx * self.precision  # m

            now_lane_id = self.global_ids[idx]
            self.splited_id = now_lane_id.split('_')[0]

            if self.splited_id == self.next_head_lane_id:
                if len(self.head_lane_ids) < 2:
                    self.prev_head_lane_id = self.now_head_lane_id
                    self.now_head_lane_id = self.next_head_lane_id
                else:
                    self.prev_head_lane_id = self.now_head_lane_id
                    self.now_head_lane_id = self.next_head_lane_id
                    self.next_head_lane_id = self.head_lane_ids[1]
                    self.head_lane_ids = self.head_lane_ids[1:]

            if self.local_path is None or (self.local_path is not None and (len(self.local_path)-self.l_idx < self.l_nitt) and len(self.erase_global_path) > self.l_cut):
                
                eg_idx = calc_idx(self.erase_global_path, (CS.position.x, CS.position.y))
                local_path = []
                local_id = []
                local_yaw = []
                local_k = []
                if len(self.erase_global_path)-eg_idx> self.l_cut:
                    if eg_idx-self.l_tail > 0:
                        local_path = self.local_path[self.l_idx-self.l_tail:]+self.erase_global_path[eg_idx+self.l_nitt:eg_idx+(self.l_cut+1)]
                        local_id = self.local_id[self.l_idx-self.l_tail:]+self.erase_global_id[eg_idx+self.l_nitt:eg_idx+(self.l_cut+1)]
                        local_yaw = local_yaw[self.l_idx-self.l_tail:]+self.erase_global_yaw[eg_idx+self.l_nitt:eg_idx+(self.l_cut+1)]
                        local_k = local_k[self.l_idx-self.l_tail:]+self.erase_global_k[eg_idx+self.l_nitt:eg_idx+(self.l_cut+1)]
                    else:
                        local_path= self.erase_global_path[eg_idx:eg_idx+(self.l_cut+1)]
                        local_id = self.erase_global_id[eg_idx:eg_idx+(self.l_cut+1)]
                        local_yaw = self.erase_global_yaw[eg_idx:eg_idx+(self.l_cut+1)]
                        local_k = self.erase_global_k[eg_idx:eg_idx+(self.l_cut+1)]
                elif len(self.global_path) < self.l_cut:
                    local_path= self.erase_global_path[eg_idx:eg_idx+(self.l_cut+1)]
                    local_id = self.erase_global_id[eg_idx:eg_idx+(self.l_cut+1)]
                    local_yaw = self.erase_global_yaw[eg_idx:eg_idx+(self.l_cut+1)]
                    local_k = self.erase_global_k[eg_idx:eg_idx+(self.l_cut+1)]
                else:
                    local_path = self.local_path[self.l_idx-self.l_tail:]+self.erase_global_path[eg_idx+self.l_nitt:]
                    local_id = self.local_id[self.l_idx-self.l_tail:]+self.erase_global_id[eg_idx+self.l_nitt:]
                    local_yaw = local_yaw[self.l_idx-self.l_tail:]+self.erase_global_yaw[eg_idx+self.l_nitt:]
                    local_k = local_k[self.l_idx-self.l_tail:]+self.erase_global_k[eg_idx+self.l_nitt:]


                self.erase_global_path = self.erase_global_path[eg_idx:]
                self.erase_global_id = self.erase_global_id[eg_idx:]
                self.erase_global_point = KDTree(self.erase_global_path)

                # self.yaw, radius, k = extract_path_info(local_path, local_id, self.lmap.lanelets)
                self.pub_local_path_theta.publish(Float32MultiArray(data=local_yaw))
                # self.pub_local_path_radius.publish(Float32MultiArray(data=radius))
                self.pub_local_path_k.publish(Float32MultiArray(data=local_k))

                self.local_path = local_path
                self.local_id = local_id
                self.l_idx = self.l_tail

            if self.local_path is not None:
                local_point = KDTree(self.local_path)
                l_idx = local_point.query((CS.position.x, CS.position.y), 1)[1]
                if abs(l_idx-self.l_idx) <= 100:
                    self.l_idx = l_idx

                splited_local_id = (self.local_id[self.l_idx]).split('_')[0]
                ### 2024.05.23 test
                ### Splited local id
                msg = String()
                msg.data = splited_local_id
                self.pub_splited_local_id.publish(msg) 


                my_neighbor_id = get_my_neighbor(self.lmap.lanelets, splited_local_id) 
                forward_direction = get_forward_direction(self.lmap.lanelets, self.now_head_lane_id, self.head_lane_ids)
                stopline_idx, stopline_wps = get_nearest_stopline(self.lmap.lanelets, self.lmap.stoplines, self.now_head_lane_id, self.head_lane_ids, local_point)

                ## Lane Change Local Signal Ver.
                if self.turnsignal != 0 and not self.turnsignal_state:
                    renew_path, renew_ids = get_lane_change_path(self.local_id, self.turnsignal, self.l_idx, self.lmap.lanelets, 
                                                                 self.local_path[self.l_idx+120:self.l_idx+240])
                    if renew_path != None:
                        for i, renew_pt in enumerate(renew_path):
                            self.local_path[self.l_idx+120+i]=renew_pt
                            self.local_id[self.l_idx+120+i]=renew_ids[i]
                        if  self.l_idx+240+30+10 < len(self.local_path)+1:
                            force_interpolate_path, _ = ref_interpolate([self.local_path[self.l_idx+120-30], self.local_path[self.l_idx+120+30]], self.precision)
                            for i, force_pt in enumerate(force_interpolate_path):
                                self.local_path[self.l_idx+120-30+i]=force_pt                  
                            force_interpolate_path, _ = ref_interpolate([self.local_path[self.l_idx+240-30], self.local_path[self.l_idx+240+30]], self.precision)
                            for i, force_pt in enumerate(force_interpolate_path):
                                self.local_path[self.l_idx+240-30+i]=force_pt
                        else:
                            print("remaining local pass is too short.")
                            pass
                    else:
                        print("The link to change lanes does not exist.")
                        pass
                    self.turnsignal_state = True
                elif self.turnsignal == 0:
                    self.turnsignal_state = False

                forward_curvature, rot_x, rot_y, trajectory = get_forward_curvature(self.l_idx, self.local_path, CS.yawRate, CS.vEgo, 0, self.lmap.lanelets, self.now_head_lane_id, self.next_head_lane_id, self.M_TO_IDX)
                # forward_curvature, rot_x, rot_y, trajectory = get_forward_curvature(self.l_idx, self.local_path, CS.yawRate, CS.vEgo, blinker, self.lmap.lanelets, self.now_head_lane_id, self.next_head_lane_id, self.M_TO_IDX)
                lane_change_point = get_lane_change_point(self.local_id, self.l_idx, my_neighbor_id)
                ## Lane Change Local Path Planning
                d = (lane_change_point - self.l_idx)*self.IDX_TO_M
                timetoarrivelanechangepoint = d/CS.vEgo if CS.vEgo != 0 else d*1000


                ## LIDAR BSD
                self.lidar_bsd = [0, 0]

                link_idx = findMyLinkIdx(self.lmap.lanelets, splited_local_id, CS.position.x, CS.position.y)
                lane_position = removeVegetationFromRoadside(self.lmap.lanelets, splited_local_id, link_idx)

                # Pubulish Lane Information
                pose = Pose()
                pose.position.x = int(splited_local_id)
                pose.position.y = get_direction_number(self.lmap.lanelets, splited_local_id, forward_direction)
                pose.position.z = stopline_idx
                pose.orientation.x = forward_curvature
                pose.orientation.y = self.l_idx
                pose.orientation.z = lane_change_point
                pose.orientation.w = lane_position
                self.pub_lane_information.publish(pose)

                local_path_viz = LocalPathViz(self.local_path)
                self.pub_local_path.publish(local_path_viz)
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

                pose = Pose()
                pose.position.x = 1
                pose.position.y = self.last_s  # m
                pose.position.z = s
                # local_path my theta
                target_heading = estimate_theta(self.local_path, self.l_idx) * 180 / np.pi
                pose.orientation.x = target_heading
                # CTE
                pose.orientation.y = calculate_cte(self.local_path[self.l_idx], self.local_path[self.l_idx+1], (CS.position.x, CS.position.y))
                self.pub_goal_object.publish(pose)
                

                # crosswalkViz
                tree = KDTree(self.lmap.lanelets[self.splited_id]['waypoints'])
                cur_id_idx = tree.query((CS.position.x, CS.position.y), 1)[1]
                global_id_idx = self.splited_global_ids.index(self.now_head_lane_id) #now_head_lane_id는 역행하지 않음
                
                
                remaining_global_ids = self.splited_global_ids[global_id_idx:] # 완

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

                # stoplineViz
                stoplinePolygonmarker = StopLineViz(stopline_wps)
                self.stoplinePolygon_pub.publish(stoplinePolygonmarker)
                
            if self.last_s - s < 5.0:
                self.state = 'ARRIVED'
            pp = 1

        elif self.state == 'ARRIVED':
            pp = 2

        return pp, self.splited_id, self.splited_global_ids, self.local_path, self.lmap, self.tmap

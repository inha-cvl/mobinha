

import rospy
import time
from scipy.spatial import KDTree
from std_msgs.msg import Int8, Float32, Float32MultiArray, Int8MultiArray, String, Int16MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Point32, Polygon
from visualization_msgs.msg import Marker


from selfdrive.planning.libs.map import LaneletMap, TileMap
from selfdrive.planning.libs.micro_lanelet_graph import MicroLaneletGraph
from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.rviz_utils import *

class PathPlanner:
    def __init__(self, CP):

        ## Common
        self.state = 'WAITING'
        self.lmap = LaneletMap(CP.mapParam.path)
        self.tmap = TileMap(self.lmap.lanelets, CP.mapParam.tileSize)


        self.graph = MicroLaneletGraph(self.lmap, CP.mapParam.cutDist).graph
        self.precision = CP.mapParam.precision
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision
        self.goal_pts = []


        ## "WAITING"
        self.get_new_goal = False


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
        self.local_id = None
        self.last_s = 99999
        self.blinker = 0
        self.blinker_target_id = None
        self.turnsignal = 0
        self.lidar_obstacle = []
        self.lidar_bsd = [0, 0]
        self.around_obstacle = []
        self.look_a_head_pos = [0, 0]
        self.nearest_obstacle_distance = -1


        ## Publisher
        self.pub_lanelet_map = rospy.Publisher('/mobinha/planning/lanelet_map', MarkerArray, queue_size=1, latch=True)
        self.map_name = rospy.get_param('map_name', 'None')
        if self.map_name == 'songdo':
            lanelet_map_viz = VectorMapVis(self.lmap.map_data)
        else:
            lanelet_map_viz = LaneletMapViz(self.lmap.lanelets, self.lmap.for_viz)

        self.pub_lanelet_map.publish(lanelet_map_viz)
        self.pub_blinker =          rospy.Publisher('/mobinha/planning/blinker', Int8, queue_size=2)
        self.pub_crosswalk_pos =    rospy.Publisher('/mobinha/planning/crosswalk_pos', Polygon, queue_size=1)
        self.pub_cte =              rospy.Publisher('/mobinha/planning/cte', Float32, queue_size=1)
        self.pub_forward_path =     rospy.Publisher('/mobinha/planning/forward_path', Marker, queue_size=1)
        self.pub_global_path =      rospy.Publisher('/mobinha/planning/global_path', Marker, queue_size=1, latch=True)
        self.pub_goal =             rospy.Publisher('/mobinha/planning/goal', Marker, queue_size=1, latch=True)
        self.pub_local_path =       rospy.Publisher('/mobinha/planning/local_path', Marker, queue_size=1)
        self.pub_lane_information = rospy.Publisher('/mobinha/planning/lane_information', Pose, queue_size=1)
        self.pub_lane_no =          rospy.Publisher('/mobinha/planning/lane_no', Int8MultiArray, queue_size=1)
        self.pub_lidar_bsd =        rospy.Publisher('/mobinha/planning/lidar_bsd', Point, queue_size=1)
        self.pub_stopline =         rospy.Publisher('/mobinha/planning/stopline', Marker, queue_size=10)
        self.pub_target_yaw =       rospy.Publisher('/mobinha/planning/target_yaw', Float32, queue_size=1)
        self.pub_path_metrics =     rospy.Publisher('/mobinha/planning/trajectory', PoseArray, queue_size=1)
        self.crosswalkPolygon_pub = rospy.Publisher('/crosswalkPolygon', MarkerArray, queue_size=10) # 정상화 TODO

        

        ## Subscriber
        rospy.Subscriber('/move_base_simple/single_goal', PoseStamped, self.single_goal_cb) # rviz 2-D nav goal
        rospy.Subscriber('/mobinha/visualize/scenario_goal',PoseArray, self.scenario_goal_cb) # scenario button from GUI
        rospy.Subscriber('/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber('/mobinha/perception/nearest_obstacle_distance', Float32, self.nearest_obstacle_distance_cb)
        rospy.Subscriber('/mobinha/perception/around_obstacle', PoseArray, self.around_obstacle_cb)
        rospy.Subscriber('/turnsignal', Int8, self.blinker_cb)
        

        # CURRENTLY NOT USING
        self.pub_local_path_theta =     rospy.Publisher('/mobinha/planning/local_path_theta', Float32MultiArray, queue_size=1)
        self.pub_local_path_radius =    rospy.Publisher('/mobinha/planning/local_path_radius', Float32MultiArray, queue_size=1)
        self.pub_local_path_k =         rospy.Publisher('/mobinha/planning/local_path_k', Float32MultiArray, queue_size=1)
        self.prevRoadPolygon_pub =      rospy.Publisher('/prevRoadPolygon', Marker, queue_size=10)
        self.nowRoadPolygon_pub =       rospy.Publisher('/nowRoadPolygon', Marker, queue_size=10)
        self.nextRoadPolygon_pub =      rospy.Publisher('/nextRoadPolygon', Marker, queue_size=10)
        rospy.Subscriber('/mobinha/control/look_ahead', Marker, self.look_a_head_cb)

    def blinker_cb(self, msg):
        self.turnsignal = msg.data

    def single_goal_cb(self, msg):
        msg_goal = [(msg.pose.position.x, msg.pose.position.y)]
        if self.goal_pts != msg_goal:
            self.goal_pts = msg_goal
            self.get_new_goal = True
            self.state = 'WAITING'

    def scenario_goal_cb(self, msg): # when scenario button clicked
        msg_goal = []
        for pose in msg.poses:
            msg_goal.append((pose.position.x, pose.position.y))
        
        if self.goal_pts != msg_goal:
            self.goal_pts = msg_goal
            self.get_new_goal = True
            print("NEW GOAL DETECTED")
            self.state = 'WAITING'

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
                self.state = 'WAITING'
                return None, None, None
            
            goal_lanelets = lanelet_matching(self.tmap.tiles, self.tmap.tile_size, goal_pt)
            if goal_lanelets is not None:
                g_id, g_idx = goal_lanelets
            else:
                rospy.logerr('Failed to match [goal] to lanelets, Insert Goal Again')
                self.state = 'WAITING'
                return None, None, None

            # 다익스트라에 사용할 정보로 변환 (waypoint -> node)
            e_node = node_matching(self.lmap.lanelets, e_id, e_idx)
            g_node = node_matching(self.lmap.lanelets, g_id, g_idx)


            if e_node == g_node:
                shortest_path = ([e_node], 0) # (path, cost)
            else:
                shortest_path = dijkstra(self.graph, e_node, g_node)
            if shortest_path is not None:
                shortest_path = shortest_path[0]
            else:
                rospy.logerr('Failed to match ego to lanelets, Insert Goal Again')
                self.state = 'WAITING'
                return None, None, None
        

            # 주행에 사용할 정보로 변환 (node -> waypoint)

            # shortest path = ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0']
            non_intp_path, non_intp_id = node_to_waypoints(self.lmap.lanelets, shortest_path)
            # non_intp_path = [[-248.78557822582127, -283.93591207290297], [-238.73866953211646, -272.1329630255155], [-229.9878153272595, -261.85324776952456], [-221.28433089503935, -251.63028019257519], [-213.18159024154102, -242.11210912113202], [-205.30837435857626, -232.8639387157932]]
            # non_intp_id = ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0']
            # 기본 형식은 'xxx_x' 리스트 형식인데, 'xxx'형식이 들어오면 node_to_waypoints에서 'xxx_x'로 형식 맞춰줌

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
    
    def initialize_planner(self):
        ## "WAITING"
        self.get_new_goal = False 

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
        self.local_id = None
        self.local_idx = 0
        self.last_s = 99999
        self.blinker = 0
        self.blinker_target_id = None
        self.turnsignal = 0
        self.lidar_obstacle = []
        self.lidar_bsd = [0, 0]
        self.around_obstacle = []
        self.look_a_head_pos = [0, 0]
        self.nearest_obstacle_distance = -1

        print("Initialized path planner!")

    def run(self, sm):
        CS = sm.CS
        pp = 0

        if self.state == 'WAITING':
            time.sleep(1)
            if self.get_new_goal:
                self.state = 'READY' 
            pp = 3

        elif self.state == 'READY': # goal 받았을 때 한 번만 실행
            start_ready = time.time()
            self.initialize_planner()
            start = time.time()
            

            non_intp_path, non_intp_id, head_lane_ids = self.returnAppendedNonIntpPath([CS.position.x, CS.position.y])
            # (x,y) 경로, 경로점에 대응하는 id, 대표 id(중복제거)
            # ex) [(x0, y0), ...], ['865_0', '865_1', '865_2', '866_0', '866_1', '781_0'], ['865', '866', '781']  
            print(f"<Elapsed:Scenario3> AppendedNonIntpPath: {(time.time() - start):.3f}")

            # Error check
            if non_intp_path is None or non_intp_id is None:
                rospy.logerr('An error occurred, unable to process path. Returning to WAITING state.')  
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


            ## Viz: Global Path
            global_path_viz = GlobalPathViz(self.global_path)
            self.pub_global_path.publish(global_path_viz)
            #---------------------------------------------------------------------------------------------------------------


            ## Viz: Goal
            goal_viz = GoalViz(self.goal_pts[-1])
            self.pub_goal.publish(goal_viz)
            #---------------------------------------------------------------------------------------------------------------


            # State 전환
            self.state = 'MOVE'

            pp = 0
            print(f"<Elapsed:Scenario3> Path Planner Total: {(time.time() - start_ready):.3f}")

        elif self.state == 'MOVE':
            global_point = KDTree(self.global_path)
            self.global_idx = global_point.query((CS.position.x, CS.position.y), 1)[1]

            self.global_idx = calc_idx(self.global_path, (CS.position.x, CS.position.y))
            if abs(self.global_idx - self.windowed_global_idx) <= 50:
                self.windowed_global_idx = self.global_idx

            s = self.windowed_global_idx * self.precision  # 진행 거리, m

            self.ego_head_id = self.global_ids[self.global_idx].split('_')[0]
            # self.ego_head_id: 현재 속한 id의 head
        

            # Local Path Generation
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

                splited_local_id = (self.local_id[self.local_idx]).split('_')[0]
                my_neighbor_id = get_my_neighbor(self.lmap.lanelets, splited_local_id) 

                
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


                ## Lane_no (hdmap:91, 1, 2, 3)
                lane_no_msg = Int8MultiArray() # HEESANG-E Haem
                prev_lane_no = -1
                now_lane_no = -1
                next_lane_no = -1
                if self.prev_head_lane_id is not None:
                    prev_lane_no = self.lmap.lanelets[self.prev_head_lane_id]['laneNo']
                if self.now_head_lane_id is not None:
                    now_lane_no = self.lmap.lanelets[self.now_head_lane_id]['laneNo']
                if self.next_head_lane_id is not None:
                    next_lane_no = self.lmap.lanelets[self.next_head_lane_id]['laneNo']
                lane_no_msg.data.append(prev_lane_no)
                lane_no_msg.data.append(now_lane_no)
                lane_no_msg.data.append(next_lane_no)
                self.pub_lane_no.publish(lane_no_msg)
                #---------------------------------------------------------------------------------------------------------------

                ## Local path
                local_path_viz = LocalPathViz(self.local_path)
                self.pub_local_path.publish(local_path_viz) 
                #---------------------------------------------------------------------------------------------------------------


                ## Blinker
                blinker, target_id = get_blinker_and_targetid(self.local_idx, self.lmap.lanelets, self.local_id, my_neighbor_id, CS.vEgo, self.M_TO_IDX, splited_local_id) 
                if blinker != 0 and self.blinker_target_id == None:
                    self.blinker_target_id = target_id
                    self.blinker = blinker
                elif splited_local_id == self.blinker_target_id:
                    self.blinker_target_id = None
                    self.blinker = 0
                self.pub_blinker.publish(blinker)
                #---------------------------------------------------------------------------------------------------------------


                ## Path metrics (curvature 관련)
                forward_curvature, rot_x, rot_y, trajectory = get_forward_curvature(self.local_idx, self.local_path, CS.yawRate, CS.vEgo, blinker, self.lmap.lanelets, self.now_head_lane_id)
                poseArray = PoseArray()
                for i in range(len(rot_x)):
                    pose = Pose()
                    pose.position.x = rot_x[i]
                    pose.position.y = rot_y[i]
                    pose.position.z = forward_curvature
                    poseArray.poses.append(pose)
                self.pub_path_metrics.publish(poseArray)
                #---------------------------------------------------------------------------------------------------------------


                ## StoplineViz
                stopline_idx, stopline_wps = get_nearest_stopline(self.lmap.lanelets, self.lmap.stoplines, self.now_head_lane_id, self.head_lane_ids, local_point)
                stopline_viz = StopLineViz(stopline_wps)
                self.pub_stopline.publish(stopline_viz)
                #---------------------------------------------------------------------------------------------------------------


                ## Forward path: TODO 이게모징
                forward_path_viz = ForwardPathViz(trajectory)
                self.pub_forward_path.publish(forward_path_viz)
                #---------------------------------------------------------------------------------------------------------------


                ## Lane information: TODO what is lane_change_point?
                forward_direction = get_forward_direction(self.lmap.lanelets, self.now_head_lane_id, self.head_lane_ids)
                link_idx = findMyLinkIdx(self.lmap.lanelets, splited_local_id, CS.position.x, CS.position.y)
                lane_change_point = get_lane_change_point(self.local_id, self.local_idx, my_neighbor_id) # What is this?
                lane_position = getLanePosition(self.lmap.lanelets, splited_local_id, link_idx)
                pose = Pose()
                pose.position.x = int(splited_local_id)
                pose.position.y = get_direction_number(self.lmap.lanelets, splited_local_id, forward_direction)
                pose.position.z = stopline_idx
                pose.orientation.x = forward_curvature
                pose.orientation.y = self.local_idx
                pose.orientation.z = lane_change_point
                pose.orientation.w = lane_position
                self.pub_lane_information.publish(pose)
                #---------------------------------------------------------------------------------------------------------------
                

                ## LIDAR BSD: TODO
                self.lidar_bsd = [0, 0]
                bsd = Point()
                bsd.x = self.lidar_bsd[0]
                bsd.y = self.lidar_bsd[1]
                self.pub_lidar_bsd.publish(bsd)
                #---------------------------------------------------------------------------------------------------------------


                ## Target yaw
                target_yaw_msg = Float32()
                target_yaw_msg.data = estimate_theta(self.local_path, self.local_idx) * 180 / np.pi
                self.pub_target_yaw.publish(target_yaw_msg)
                #---------------------------------------------------------------------------------------------------------------


                ## CTE
                cte_msg = Float32()
                cte_msg.data = calculate_cte(self.local_path[self.local_idx], self.local_path[self.local_idx+1], (CS.position.x, CS.position.y))
                self.pub_cte.publish(cte_msg)
                #---------------------------------------------------------------------------------------------------------------
                

                ## CrosswalkViz: TODO
                tree = KDTree(self.lmap.lanelets[self.ego_head_id]['waypoints'])
                cur_id_idx = tree.query((CS.position.x, CS.position.y), 1)[1]
                global_id_idx = self.global_head_ids.index(self.now_head_lane_id) #now_head_lane_id는 역행하지 않음
                
                
                remaining_global_ids = self.global_head_ids[global_id_idx:]

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
                #---------------------------------------------------------------------------------------------------------------

                

                
                
            if self.last_s - s < 5.0:
                self.state = 'ARRIVED'
            pp = 1

        elif self.state == 'ARRIVED':
            pp = 2

        return pp, self.ego_head_id, self.global_head_ids, self.local_path, self.lmap, self.tmap
# pp = 4는 TOR (판단불능->제어권 넘김)
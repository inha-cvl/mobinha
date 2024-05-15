

import rospy
import time
from scipy.spatial import KDTree
from std_msgs.msg import Int8, Float32, Float32MultiArray, Int8MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from visualization_msgs.msg import Marker

from selfdrive.planning.libs.map import LaneletMap, TileMap
from selfdrive.planning.libs.micro_lanelet_graph import MicroLaneletGraph
from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.rviz_utils import *


class PathPlanner:
    def __init__(self, CP):
        self.state = 'WAITING'
        self.lmap = LaneletMap(CP.mapParam.path)
        self.tmap = TileMap(self.lmap.lanelets, CP.mapParam.tileSize)
        self.graph = MicroLaneletGraph(self.lmap, CP.mapParam.cutDist).graph
        self.precision = CP.mapParam.precision
        self.M_TO_IDX = 1/CP.mapParam.precision
        self.IDX_TO_M = CP.mapParam.precision

        self.temp_pt = None
        self.global_path = None
        self.global_id = None
        self.erase_global_point = None
        self.non_intp_path = None
        self.non_intp_id = None
        self.local_path = None
        self.local_id = None
        self.temp_global_idx = 0
        self.local_path_theta = None
        self.yaw = None
        self.prev_yaw = None
        # self.global_yaw, self.global_k = [], []

        self.get_goal = False

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

        self.pub_lanelet_map = rospy.Publisher('/mobinha/planning/lanelet_map', MarkerArray, queue_size=1, latch=True)
        self.pub_goal_viz = rospy.Publisher('/mobinha/planning/goal_viz', Marker, queue_size=1, latch=True)
        self.pub_global_path = rospy.Publisher('/mobinha/global_path', Marker, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher('/mobinha/planning/local_path', Marker, queue_size=1)
        self.pub_blinkiker = rospy.Publisher('/mobinha/planning/blinker', Int8, queue_size=2)
        self.pub_goal_object = rospy.Publisher('/mobinha/planning/goal_information', Pose, queue_size=1)
        self.pub_forward_path = rospy.Publisher('/mobinha/planning/forward_path', Marker, queue_size=1)
        self.pub_lane_information = rospy.Publisher('/mobinha/planning/lane_information', Pose, queue_size=1)
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
        self.pub_right_turn_situation = rospy.Publisher('/mobinha/planning/right_turn_situation_real', Int8MultiArray, queue_size=1)
        map_name = rospy.get_param('map_name', 'None')
        if map_name == 'songdo':
            lanelet_map_viz = VectorMapVis(self.lmap.map_data)
        else:
            lanelet_map_viz = LaneletMapViz(self.lmap.lanelets, self.lmap.for_viz)
        self.pub_lanelet_map.publish(lanelet_map_viz)

        rospy.Subscriber('/move_base_simple/single_goal', PoseStamped, self.single_goal_cb)
        rospy.Subscriber('/mobinha/visualize/scenario_goal',PoseArray, self.scenario_goal_cb)
        rospy.Subscriber('/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber('/mobinha/perception/nearest_obstacle_distance', Float32, self.nearest_obstacle_distance_cb)
        rospy.Subscriber('/mobinha/control/look_ahead', Marker, self.look_a_head_cb)
        rospy.Subscriber('/mobinha/perception/around_obstacle', PoseArray, self.around_obstacle_cb)
        rospy.Subscriber('/turnsignal', Int8, self.blinker_cb)

    def blinker_cb(self, msg):
        self.turnsignal = msg.data

    def single_goal_cb(self, msg):
        self.goal_pts = [(msg.pose.position.x, msg.pose.position.y)]
        self.get_goal = True

    def scenario_goal_cb(self, msg):
        scenario_goal = []
        for pose in msg.poses:
            scenario_goal.append((pose.position.x, pose.position.y))
        self.goal_pts = scenario_goal
        self.get_goal = True

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

            while True:
                ego_lanelets = lanelet_matching(
                    self.tmap.tiles, self.tmap.tile_size, self.temp_pt)
                if ego_lanelets is not None:
                    e_id, e_idx = ego_lanelets
                else:
                    rospy.logerr('Failed to match ego to lanelets, Insert Goal Again')
                    self.get_goal = 0
                    self.state = 'WAITING'
                    return None, None, None

                goal_lanelets = lanelet_matching(self.tmap.tiles, self.tmap.tile_size, goal_pt)
                if goal_lanelets is not None:
                    g_id, g_idx = goal_lanelets

                else:
                    rospy.logerr('Failed to match ego to lanelets, Insert Goal Again')
                    self.get_goal = 0
                    self.state = 'WAITING'
                    return None, None, None

                e_node = node_matching(self.lmap.lanelets, e_id, e_idx)
                g_node = node_matching(self.lmap.lanelets, g_id, g_idx)

                if e_node == g_node:
                    shortest_path = ([e_node], 0)
                else:
                    shortest_path = dijkstra(self.graph, e_node, g_node)

                if shortest_path is not None:
                    shortest_path = shortest_path[0]
                    break
                else:
                    rospy.logerr('Failed to match ego to lanelets, Insert Goal Again')
                    self.get_goal = 0
                    self.state = 'WAITING'
                    return None, None

            non_intp_path, non_intp_id = node_to_waypoints2(self.lmap.lanelets, shortest_path)
            intp_start_idx = calc_idx(non_intp_path, self.temp_pt)
            intp_last_idx = calc_idx(non_intp_path, goal_pt)

            non_intp_path = non_intp_path[intp_start_idx:intp_last_idx+1]
            non_intp_id = non_intp_id[intp_start_idx:intp_last_idx+1]

            appended_non_intp_path.extend(non_intp_path)
            appended_non_intp_id.extend(non_intp_id)
            appended_head_lane_ids.extend(shortest_path)

            self.temp_pt = goal_pt

        appended_head_lane_ids = set_lane_ids(appended_head_lane_ids)

        goal_viz = GoalViz(goal_pt)
        self.pub_goal_viz.publish(goal_viz)

        return appended_non_intp_path, appended_non_intp_id, appended_head_lane_ids

    def delete_node_for_smooth_path(self, non_intp_path, non_intp_id, head_lane_ids):
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
            if self.get_goal != 0:
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

            non_intp_path, non_intp_id, head_lane_ids = self.returnAppendedNonIntpPath()
            if non_intp_path is None or non_intp_id is None:
                rospy.logerr('An error occurred, unable to process path. Returning to WAITING state.')
                self.state = 'WAITING'
                self.get_goal = 0
                pp = 3
                return pp, None

            self.delete_node_for_smooth_path(non_intp_path, non_intp_id, head_lane_ids)
            if non_intp_path is not None:
                self.state = 'MOVE'

                global_path, self.last_s = ref_interpolate_2d(non_intp_path, self.precision)
                global_path, global_yaw, global_k = smooth_compute_yaw_and_curvature(global_path, self.precision)
                global_id = id_interpolate(non_intp_path, global_path, non_intp_id)
                self.global_path = global_path
                self.global_id = global_id
                self.non_intp_path = non_intp_path
                self.non_intp_id = non_intp_id
                self.head_lane_ids = head_lane_ids

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
                self.erase_global_id = global_id
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

            now_lane_id = self.global_id[idx]
            splited_id = now_lane_id.split('_')[0]

            if splited_id == self.next_head_lane_id:
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
                my_neighbor_id = get_my_neighbor(self.lmap.lanelets, splited_local_id) 
                forward_direction = get_forward_direction(self.lmap.lanelets, self.now_head_lane_id, self.head_lane_ids)
                stopline_s, stopline_wps = get_nearest_stopline(self.lmap.lanelets, self.lmap.stoplines, self.now_head_lane_id, self.head_lane_ids, local_point)

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

                blinker, target_id = get_blinker(self.l_idx, self.lmap.lanelets, self.local_id, my_neighbor_id, CS.vEgo, self.M_TO_IDX, splited_local_id) 
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

                forward_curvature, rot_x, rot_y, trajectory = get_forward_curvature(self.l_idx, self.local_path, CS.yawRate, CS.vEgo, blinker, self.lmap.lanelets, self.now_head_lane_id, self.next_head_lane_id, self.M_TO_IDX)
                lane_change_point = get_lane_change_point(self.local_id, self.l_idx, my_neighbor_id)
                ## Lane Change Local Path Planning
                d = (lane_change_point - self.l_idx)*self.IDX_TO_M
                timetoarrivelanechangepoint = d/CS.vEgo if CS.vEgo != 0 else d*1000
                self.lidar_bsd = [0, 0]

                if blinker != 0 and not self.renewal_path_in_progress:
                    # look a head's idx's id == lane id => stop looking BSD
                    look_a_head_idx = local_point.query(self.look_a_head_pos, 1)[1]
                    look_a_head_id = self.local_id[look_a_head_idx].split('_')[0]
                    get_look_a_head_id = compare_id(look_a_head_id, my_neighbor_id)
                    renew_a = 30 # uniti: idx
                    renew_b = 120 # unit : idx
                    for obs in self.around_obstacle:
                        print(obs)
                        print(get_look_a_head_id)
                        #Left
                        if blinker == 1 and get_look_a_head_id and -4.05<obs[2]<-1.8 and lane_change_point<(len(self.local_path)-1): # frenet d coordinate left. 
                            #TODO: if left lane change, get prev,now,next leftBound and check obstacle where is it. 
                            _, _, _, isCarInRoad = is_car_inside_combined_road((obs[3],obs[4]),self.lmap.lanelets, self.prev_head_lane_id, self.now_head_lane_id, self.next_head_lane_id)
                            if isCarInRoad:
                                vTargetCar = (obs[5] + CS.vEgo) # unit: m/s
                                targetcarmovingdistance = vTargetCar * timetoarrivelanechangepoint # unit: m
                                safedistance = vTargetCar*MPS_TO_KPH - 15 # unit: m 
                                print("d: ", d)
                                print("targetmove: ", targetcarmovingdistance)
                                print(safedistance)
                                print("obs distance:",(obs[1] - self.l_idx)*self.IDX_TO_M)
                                if safedistance < 10:
                                    safedistance = 10 # 5 * 2 : front and back 
                                safe_space = (safedistance/2)
                                obs_distance = (obs[1] - self.l_idx)*self.IDX_TO_M
                                if targetcarmovingdistance + obs_distance - safe_space < d < targetcarmovingdistance + obs_distance + safe_space:
                                    #get renewable local path
                                    renew_path, renew_ids = get_renew_path(self.local_id, blinker, lane_change_point, self.lmap.lanelets, 
                                                                        self.local_path[lane_change_point:lane_change_point+renew_b], self.local_path[lane_change_point-renew_a:lane_change_point])
                                    self.lidar_bsd = [1, 0]
                                    if renew_path != None:
                                        for i, renew_pt in enumerate(renew_path):
                                            self.local_path[lane_change_point-renew_a+i]=renew_pt
                                            self.local_id[lane_change_point-renew_a+i]=renew_ids[i]
                                        if  lane_change_point+renew_a+renew_b+25 < len(self.local_path)+1:
                                            force_interpolate_path,_ = ref_interpolate([self.local_path[lane_change_point-renew_a+renew_b], self.local_path[lane_change_point+renew_a+renew_b]], self.precision)
                                            print("left BSD")
                                            for i, force_pt in enumerate(force_interpolate_path):
                                                self.local_path[lane_change_point-renew_a+renew_b+i]=force_pt
                                            self.renewal_path_in_progress = True
                                            self.renewal_path_cnt += 1
                                            self.renewal_path_timer = time.time()
                                            break # multi obstacle passing
                                        else:
                                            pass
                                    else:
                                        print("Take Over Request")
                                        pp = 4
                                        if pp == 4:
                                            self.renewal_path_cnt += 1
                                        if self.renewal_path_cnt > 30:
                                            self.renewal_path_cnt = 0
                                        return pp, self.local_path
                        elif blinker == 2 and get_look_a_head_id and 1.8<obs[2]<4.05 and lane_change_point<(len(self.local_path)-1): # frenet d coordinate right.
                            _, _, _, isCarInRoad = is_car_inside_combined_road((obs[3],obs[4]),self.lmap.lanelets, self.prev_head_lane_id, self.now_head_lane_id, self.next_head_lane_id)
                            if isCarInRoad:
                                vTargetCar = (obs[5] + CS.vEgo) # unit: m/s
                                targetcarmovingdistance = vTargetCar * timetoarrivelanechangepoint # unit: m
                                safedistance = vTargetCar*MPS_TO_KPH - 15 # unit: m 
                                print("d: ", d)
                                print("targetmove: ", targetcarmovingdistance)
                                print(safedistance)
                                print("obs distance:",(obs[1] - self.l_idx)*self.IDX_TO_M)
                                if safedistance < 10:
                                    safedistance = 10 # 5 * 2 : front and back 
                                safe_space = (safedistance/2)
                                obs_distance = (obs[1] - self.l_idx)*self.IDX_TO_M
                                if targetcarmovingdistance + obs_distance - safe_space < d < targetcarmovingdistance + obs_distance + safe_space:
                                    
                                    #get renewable local path
                                    renew_path, renew_ids = get_renew_path(self.local_id, blinker, lane_change_point, self.lmap.lanelets, 
                                                                        self.local_path[lane_change_point:lane_change_point+renew_b], self.local_path[lane_change_point-renew_a:lane_change_point])
                                    self.lidar_bsd = [0, 1]
                                    if renew_path != None:
                                        for i, renew_pt in enumerate(renew_path):
                                            self.local_path[lane_change_point-renew_a+i]=renew_pt
                                            self.local_id[lane_change_point-renew_a+i]=renew_ids[i]
                                        if  lane_change_point+renew_a+renew_b+25 < len(self.local_path)+1:
                                            force_interpolate_path,_ = ref_interpolate([self.local_path[lane_change_point-renew_a+renew_b], self.local_path[lane_change_point+renew_a+renew_b]], self.precision)
                                            print("right BSD")
                                            for i, force_pt in enumerate(force_interpolate_path):
                                                self.local_path[lane_change_point-renew_a+renew_b+i]=force_pt
                                            self.renewal_path_in_progress = True
                                            self.renewal_path_cnt += 1
                                            self.renewal_path_timer = time.time()
                                            break
                                        else:
                                            pass
                                    else:
                                        print("Take Over Request")
                                        pp = 4
                                        if pp == 4:
                                            self.renewal_path_cnt += 1
                                        if self.renewal_path_cnt > 30:
                                            self.renewal_path_cnt = 0
                                        return pp, self.local_path
                elif self.renewal_path_cnt >= 2:
                    print("Take Over Request(continuos 2 times)")
                    pp = 4
                    if pp == 4:
                        self.renewal_path_cnt += 1
                    if self.renewal_path_cnt > 30:
                        self.renewal_path_cnt = 0
                    return pp, self.local_path

                elif time.time() - self.renewal_path_timer > 1.5:
                    self.renewal_path_in_progress = False

                # TODO: Avoidance Path
                #
                # if -1 < self.nearest_obstacle_distance and self.nearest_obstacle_distance <= 12.0 and len(self.lidar_obstacle) >= 0:
                #     if self.obstacle_detect_timer == 0.0:
                #         self.obstacle_detect_timer = time.time()
                #     if time.time()-self.obstacle_detect_timer >= 10:
                #         # pp = 4
                #         # return pp
                #         # Create Avoidance Trajectory
                #         splited_id = now_lane_id.split('_')[0]
                #         avoid_path = generate_avoid_path(
                #             self.lmap.lanelets, splited_id, self.local_path[self.l_idx:], 25*(1/self.precision))
                #         if avoid_path is not None:
                #             for i, avoid_pt in enumerate(avoid_path):
                #                 self.local_path[self.l_idx+i] = avoid_pt
                #             self.obstacle_detect_timer = 0.0
                # else:
                #     self.obstacle_detect_timer = 0.0

                # CHECK : now lane position
                '''
                [1] |@| | |
                [2] | |@| |
                [3] | | |@|
                [0]  x|@|x  
                '''
                link_idx = findMyLinkIdx(self.lmap.lanelets, splited_local_id, CS.position.x, CS.position.y)
                lane_position = removeVegetationFromRoadside(self.lmap.lanelets, splited_local_id, link_idx)

                # Pubulish Lane Information
                pose = Pose()
                pose.position.x = int(splited_local_id)
                pose.position.y = get_direction_number(self.lmap.lanelets, splited_local_id, forward_direction)
                pose.position.z = stopline_s
                pose.orientation.x = forward_curvature
                pose.orientation.y = self.l_idx
                pose.orientation.z = lane_change_point
                pose.orientation.w = lane_position
                self.pub_lane_information.publish(pose)

                local_path_viz = LocalPathViz(self.local_path)
                self.pub_local_path.publish(local_path_viz)

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
                target_heading = estimate_theta(self.local_path, self.l_idx) * 180 / np.pi
                pose.orientation.x = target_heading
                # CTE
                pose.orientation.y = calculate_cte(self.local_path[self.l_idx], self.local_path[self.l_idx+1], (CS.position.x, CS.position.y))
                self.pub_goal_object.publish(pose)

                # crosswalkViz
                crosswalk_ids, crosswalkPoints = get_crosswalk_points(self.lmap.lanelets, self.lmap.surfacemarks, self.now_head_lane_id, self.head_lane_ids)
                crosswalkPolygonmarker = CrosswalkViz(crosswalkPoints)
                self.crosswalkPolygon_pub.publish(crosswalkPolygonmarker)

                if is_obstacle_inside_polygon(self.lmap.surfacemarks, crosswalk_ids, self.around_obstacle):
                    self.pub_right_turn_situation.publish(Int8MultiArray(data=[0,1]))
                else:
                    self.pub_right_turn_situation.publish(Int8MultiArray(data=[0,0]))

                # stoplineViz
                stoplinePolygonmarker = StopLineViz(stopline_wps)
                self.stoplinePolygon_pub.publish(stoplinePolygonmarker)
                
            if self.last_s - s < 5.0:
                self.state = 'ARRIVED'
            pp = 1

        elif self.state == 'ARRIVED':
            pp = 2

        return pp, self.local_path
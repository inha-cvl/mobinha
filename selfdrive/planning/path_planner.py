

import rospy
import time
from scipy.spatial import KDTree
from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

from selfdrive.planning.libs.map import LaneletMap, TileMap
from selfdrive.planning.libs.micro_lanelet_graph import MicroLaneletGraph
from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.viz_utils import *


class PathPlanner:
    def __init__(self, CP):
        self.state = 'WAITING'
        self.lmap = LaneletMap(CP.mapParam.path)
        self.tmap = TileMap(self.lmap.lanelets, CP.mapParam.tileSize)
        self.graph = MicroLaneletGraph(self.lmap, CP.mapParam.cutDist).graph
        self.precision = CP.mapParam.precision

        self.temp_pt = None
        self.global_path = None
        self.erase_global_point = None
        self.non_intp_path = None
        self.non_intp_id = None
        self.local_path = None
        self.temp_global_idx = 0

        self.get_goal = False

        self.l_idx = 0
        self.erase_global_path = []
        self.last_s = 99999

        self.lidar_obstacle = []
        self.obstacle_detect_timer = 0
        self.nearest_obstacle_distance = -1

        self.pub_lanelet_map = rospy.Publisher(
            '/mobinha/planning/lanelet_map', MarkerArray, queue_size=1, latch=True)
        self.pub_goal_viz = rospy.Publisher(
            '/mobinha/planning/goal_viz', Marker, queue_size=1, latch=True)
        self.pub_global_path = rospy.Publisher(
            '/mobinha/global_path', Marker, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher(
            '/mobinha/planning/local_path', Marker, queue_size=1)
        self.pub_blinkiker = rospy.Publisher(
            '/mobinha/planning/blinker', Int8, queue_size=2)
        self.pub_goal_object = rospy.Publisher(
            '/mobinha/planning/goal_information', Pose, queue_size=1)
        self.pub_forward_direction = rospy.Publisher(
            '/mobinha/planning/forward_direction', Int8, queue_size=1)
        self.pub_forward_path = rospy.Publisher(
            '/mobinha/planning/forward_path', Marker, queue_size=1)
        self.pub_lane_information = rospy.Publisher(
            '/mobinha/planning/lane_information', Pose, queue_size=1)

        map_name = rospy.get_param('map_name', 'None')
        if map_name == 'songdo':
            lanelet_map_viz = VectorMapVis(self.lmap.map_data)
        else:
            lanelet_map_viz = LaneletMapViz(
                self.lmap.lanelets, self.lmap.for_viz)
        self.pub_lanelet_map.publish(lanelet_map_viz)

        rospy.Subscriber(
            '/move_base_simple/single_goal', PoseStamped, self.single_goal_cb)
        rospy.Subscriber('/mobinha/visualize/scenario_goal',
                         PoseArray, self.scenario_goal_cb)
        rospy.Subscriber(
            '/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber(
            '/mobinha/perception/nearest_obstacle_distance', Float32, self.nearest_obstacle_distance_cb)

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
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z)
                               for pose in msg.poses]

    def nearest_obstacle_distance_cb(self, msg):
        self.nearest_obstacle_distance = round(msg.data, 5)  # nearest obstacle

    def returnAppendedNonIntpPath(self):
        appended_non_intp_path = []
        appended_non_intp_id = []
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
                    rospy.logerr(
                        'Failed to match ego to lanelets, Insert Goal Again')
                    self.get_goal = 0
                    self.state = 'WAITING'
                    return None, None

                goal_lanelets = lanelet_matching(
                    self.tmap.tiles, self.tmap.tile_size, goal_pt)
                if goal_lanelets is not None:
                    g_id, g_idx = goal_lanelets

                else:
                    rospy.logerr(
                        'Failed to match ego to lanelets, Insert Goal Again')
                    self.get_goal = 0
                    self.state = 'WAITING'
                    return None, None

                e_node = node_matching(self.lmap.lanelets, e_id, e_idx)
                g_node = node_matching(
                    self.lmap.lanelets, g_id, g_idx)

                if e_node == g_node:
                    shortest_path = ([e_node], 0)
                else:
                    shortest_path = dijkstra(self.graph, e_node, g_node)

                if shortest_path is not None:
                    shortest_path = shortest_path[0]
                    break
                else:
                    rospy.logerr(
                        'Failed to match ego to lanelets, Insert Goal Again')
                    self.get_goal = 0
                    self.state = 'WAITING'
                    return None, None
            non_intp_path, non_intp_id = node_to_waypoints2(
                self.lmap.lanelets, shortest_path)
            intp_start_idx = calc_idx(non_intp_path, self.temp_pt)
            intp_last_idx = calc_idx(non_intp_path, goal_pt)

            non_intp_path = non_intp_path[intp_start_idx:intp_last_idx+1]
            non_intp_id = non_intp_id[intp_start_idx:intp_last_idx+1]

            appended_non_intp_path.extend(non_intp_path)
            appended_non_intp_id.extend(non_intp_id)

            self.temp_pt = goal_pt

        goal_viz = GoalViz(goal_pt)
        self.pub_goal_viz.publish(goal_viz)

        return appended_non_intp_path, appended_non_intp_id

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
            self.local_path = None
            self.temp_global_idx = 0
            self.l_idx = 0
            self.local_path_cut_value = 500
            self.local_path_nitting_value = 150
            self.local_path_tail_value = 50
            self.temp_pt = [CS.position.x, CS.position.y]

            non_intp_path, non_intp_id = self.returnAppendedNonIntpPath()
            if non_intp_path == None and non_intp_id == None:
                pass
            if non_intp_path is not None:
                self.state = 'MOVE'
                global_path, self.last_s = ref_interpolate(
                    non_intp_path, self.precision, 0, 0)

                self.global_path = global_path
                self.non_intp_path = non_intp_path
                self.non_intp_id = non_intp_id
                self.erase_global_path = global_path
                global_path_viz = FinalPathViz(self.global_path)
                self.pub_global_path.publish(global_path_viz)

            pp = 0

        elif self.state == 'MOVE':
            idx = calc_idx(
                self.global_path, (CS.position.x, CS.position.y))

            if abs(idx-self.temp_global_idx) <= 50:
                self.temp_global_idx = idx

            s = self.temp_global_idx * self.precision  # m
            n_id = calc_idx(
                self.non_intp_path, (CS.position.x, CS.position.y))
            now_lane_id = self.non_intp_id[n_id]

            if self.local_path is None or (self.local_path is not None and (len(self.local_path)-self.l_idx < self.local_path_nitting_value) and len(self.local_path) > self.local_path_nitting_value):

                eg_idx = calc_idx(
                    self.erase_global_path, (CS.position.x, CS.position.y))
                local_path = []
                if len(self.erase_global_path)-eg_idx > self.local_path_cut_value:
                    if eg_idx-self.local_path_tail_value > 0:
                        local_path = self.erase_global_path[eg_idx-self.local_path_tail_value:eg_idx +
                                                            self.local_path_cut_value]
                    else:
                        local_path = self.erase_global_path[eg_idx:eg_idx +
                                                            self.local_path_cut_value]
                else:
                    if eg_idx-self.local_path_tail_value > 0:
                        local_path = self.erase_global_path[eg_idx -
                                                            self.local_path_tail_value:]
                    else:
                        local_path = self.erase_global_path[eg_idx:]

                self.erase_global_path = self.erase_global_path[eg_idx:]
                self.erase_global_point = KDTree(self.erase_global_path)
                self.local_path = local_path

            if self.local_path is not None:

                local_point = KDTree(self.local_path)
                self.l_idx = local_point.query(
                    (CS.position.x, CS.position.y), 1)[1]
                splited_id = now_lane_id.split('_')[0]

                erase_idx = self.erase_global_point.query(
                    (CS.position.x, CS.position.y), 1)[1]
                forward_direction, forward_path = get_forward_direction(
                    self.erase_global_path, erase_idx)
                self.pub_forward_direction.publish(forward_direction)
                # for visualize
                forward_path_viz = ForwardPathViz(forward_path)
                self.pub_forward_path.publish(forward_path_viz)

                cw_s = get_nearest_crosswalk(
                    self.lmap.lanelets, splited_id, local_point)
                pose = Pose()
                pose.position.x = int(splited_id)
                # straight, left, right, l-change, r-change, uturn
                pose.position.y = forward_direction
                pose.position.z = cw_s
                self.pub_lane_information.publish(pose)

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
                #             self.lmap.lanelets, splited_id, self.local_path[self.l_idx:], 25)
                #         if avoid_path is not None:
                #             for i, avoid_pt in enumerate(avoid_path):
                #                 self.local_path[self.l_idx+i] = avoid_pt
                #             self.obstacle_detect_timer = 0.0
                # else:
                #     self.obstacle_detect_timer = 0.0

                local_path_viz = LocalPathViz(self.local_path)
                self.pub_local_path.publish(local_path_viz)

            blinker = get_blinker(self.lmap.lanelets, self.non_intp_id, n_id)
            self.pub_blinkiker.publish(blinker)

            pose = Pose()
            pose.position.x = 1
            pose.position.y = self.last_s  # m
            pose.position.z = s
            self.pub_goal_object.publish(pose)

            if self.last_s - s < 5.0:
                self.state = 'ARRIVED'
            pp = 1

        elif self.state == 'ARRIVED':
            pp = 2

        return pp, self.local_path

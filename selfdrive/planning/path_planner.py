

import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PoseStamped

from libs.map import LaneletMap, TileMap
from libs.micro_lanelet_graph import MicroLaneletGraph
from libs.planner_utils import *
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
        self.non_intp_path = None
        self.non_intp_id = None

        self.local_path = None
        self.temp_global_idx = 0

        self.get_goal = False

        self.l_idx = 0
        self.erase_global_path = []
        self.last_s = 99999

        self.pub_lanelet_map = rospy.Publisher(
            '/lanelet_map', MarkerArray, queue_size=1, latch=True)
        self.pub_goal = rospy.Publisher(
            '/goal', Marker, queue_size=1, latch=True)
        self.pub_global_path = rospy.Publisher(
            '/global_path', Marker, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher(
            '/local_path', Marker, queue_size=1)
        self.pub_now_lane_id = rospy.Publisher(
            '/now_lane_id', String, queue_size=1)
        self.pub_blinkiker = rospy.Publisher(
            '/lane_change', Int8, queue_size=2)

        lanelet_map_viz = LaneletMapViz(self.lmap.lanelets, self.lmap.for_viz)
        self.pub_lanelet_map.publish(lanelet_map_viz)

        self.sub_goal = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.goal_cb)

    def goal_cb(self, msg):
        self.goal_pt = [msg.pose.position.x, msg.pose.position.y]
        self.get_goal = True

    def returnAppendedNonIntpPath(self, goal_pt):
        shortest_path = []

        g_id = None
        g_idx = None

        while True:
            ego_lanelets = lanelet_matching(
                self.tmap.tiles, self.tmap.tile_size, self.temp_pt)
            if ego_lanelets is not None:
                e_id, e_idx = ego_lanelets
            else:
                rospy.logerr('Failed to match ego to lanelets!')
                continue

            goal_lanelets = lanelet_matching(
                self.tmap.tiles, self.tmap.tile_size, goal_pt)
            if goal_lanelets is not None:
                g_id, g_idx = goal_lanelets
                goal_viz = GoalViz(goal_pt)
                self.pub_goal.publish(goal_viz)
            else:
                rospy.logerr('Failed to match goal to lanelets!')
                continue

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
                rospy.logerr('Failed to find shortest path!')
                continue

        non_intp_path, non_intp_id = node_to_waypoints2(
            self.lmap.lanelets, shortest_path)
        _, intp_start_idx = calc_cte_and_idx(non_intp_path, self.temp_pt)
        _, intp_last_idx = calc_cte_and_idx(non_intp_path, goal_pt)

        non_intp_path = non_intp_path[intp_start_idx:intp_last_idx+1]
        non_intp_id = non_intp_id[intp_start_idx:intp_last_idx+1]

        return non_intp_path, non_intp_id

    def run(self, sm):
        CS = sm.CS
        pp = 0

        if self.state == 'WAITING':
            rospy.loginfo("[Path Planner] Waiting Goal Point")
            time.sleep(1)
            if self.get_goal:
                self.state = 'READY'

        elif self.state == 'READY':
            rospy.loginfo("[Path Planner] Making Path")
            non_intp_path = None
            non_intp_id = None
            self.local_path = None
            self.temp_global_idx = 0
            self.l_idx = 0
            self.min_v = 0.5

            self.temp_pt = [CS.position.x, CS.position.y]

            non_intp_path, non_intp_id = self.returnAppendedNonIntpPath(
                self.goal_pt)

            if non_intp_path is not None:
                self.state = 'MOVE'
                rospy.loginfo("[Planner] Move to Goal")
                global_path, _, self.last_s = ref_interpolate(
                    non_intp_path, self.precision, 0, 0)

                # For Normal Arrive
                last_idx_of_global = len(global_path)-1
                x_increment = (
                    global_path[last_idx_of_global][0]-global_path[last_idx_of_global-1][0])
                y_increment = (
                    global_path[last_idx_of_global][1]-global_path[last_idx_of_global-1][1])
                while True:
                    global_path.append(
                        [(global_path[last_idx_of_global][0]+x_increment), (global_path[last_idx_of_global][1]+y_increment)])
                    x_increment += x_increment
                    y_increment += y_increment
                    if x_increment >= 5 or y_increment >= 5:
                        break

                self.global_path = global_path
                self.non_intp_path = non_intp_path
                self.non_intp_id = non_intp_id
                self.erase_global_path = global_path
                global_path_viz = FinalPathViz(self.global_path)
                self.pub_global_path.publish(global_path_viz)
            pp = 0

        elif self.state == 'MOVE':
            _, idx = calc_cte_and_idx(
                self.global_path, (CS.position.x, CS.position.y))

            if abs(idx-self.temp_global_idx) <= 50:
                self.temp_global_idx = idx

            s = idx * self.precision
            _, n_id = calc_cte_and_idx(
                self.non_intp_path, (CS.position.x, CS.position.y))

            # Pub Now Lane ID
            self.pub_now_lane_id.publish(String(self.non_intp_id[n_id]))

            if self.local_path is None or (self.local_path is not None and (len(self.local_path)-self.l_idx < 100) and len(self.local_path) > 100):

                _, eg_idx = calc_cte_and_idx(
                    self.erase_global_path, (CS.position.x, CS.position.y))
                local_path = []
                if len(self.erase_global_path)-eg_idx > 500:
                    local_path = self.erase_global_path[eg_idx:eg_idx+500]
                else:
                    local_path = self.erase_global_path[eg_idx:]
                self.erase_global_path = self.erase_global_path[eg_idx:]

                self.local_path = local_path

            if self.local_path is not None:

                _, self.l_idx = calc_cte_and_idx(
                    self.local_path, (CS.position.x, CS.position.y))

                local_path_viz = LocalPathViz(self.local_path)
                self.pub_local_path.publish(local_path_viz)

            blinker = signal_light_toggle(
                self.non_intp_path, n_id, self.precision,  self.tmap, self.lmap, 1)
            self.pub_blinkiker.publish(blinker)

            if self.last_s - s < 4.0:
                self.state = 'ARRIVED'
                rospy.loginfo('[Path Planner] Arrived at Goal')
            pp = 1

        elif self.state == 'ARRIVED':
            pp = 2

        return pp

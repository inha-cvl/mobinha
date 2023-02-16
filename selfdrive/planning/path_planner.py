

import rospy
import time
import math
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
            #print("[{}] Waiting Goal Point".format(self.__class__.__name__))
            time.sleep(1)
            if self.get_goal != 0:
                self.state = 'READY'
            pp = 3

        elif self.state == 'READY':
            #print("[{}] Making Path".format(self.__class__.__name__))
            non_intp_path = None
            non_intp_id = None
            self.local_path = None
            self.temp_global_idx = 0
            self.l_idx = 0
            self.local_path_cut_value = 300
            self.local_path_nitting_value = 150
            self.local_path_tail_value = 50
            self.temp_pt = [CS.position.x, CS.position.y]

            non_intp_path, non_intp_id = self.returnAppendedNonIntpPath()
            non_intp_path = []
            non_intp_id = [0]

            if non_intp_path == None and non_intp_id == None:
                pass

            if non_intp_path is not None:
                self.state = 'MOVE'
                #print("[{}] Move to Goal".format(self.__class__.__name__))
                # global_path, self.last_s = ref_interpolate(
                    # non_intp_path, self.precision, 0, 0)
                
                global_path = [
				[
					-0.17199644166521466,
					-0.07689262548367726
				],
				[
					-1.0590557294960716,
					-0.5385481462705652
				],
				[
					-1.9461150173269286,
					-1.0002036670574532
				],
				[
					-2.833174305157786,
					-1.4618591878443414
				],
				[
					-3.7202335929886425,
					-1.9235147086312294
				],
				[
					-4.607292880819499,
					-2.3851702294181174
				],
				[
					-5.494352168650357,
					-2.8468257502050056
				],
				[
					-6.381411456481214,
					-3.3084812709918934
				],
				[
					-7.26847074431207,
					-3.7701367917787816
				],
				[
					-8.155530032142927,
					-4.23179231256567
				],
				[
					-9.042589319973784,
					-4.6934478333525576
				],
				[
					-9.92964860780464,
					-5.155103354139445
				],
				[
					-10.8167078956355,
					-5.616758874926334
				],
				[
					-11.703767183466356,
					-6.078414395713222
				],
				[
					-12.590826471297213,
					-6.5400699165001095
				],
				[
					-13.47788575912807,
					-7.001725437286998
				],
				[
					-14.364945046958926,
					-7.463380958073886
				],
				[
					-15.252004334789783,
					-7.925036478860774
				],
				[
					-16.13906362262064,
					-8.386691999647663
				],
				[
					-17.026122910451498,
					-8.848347520434551
				],
				[
					-17.913182198282353,
					-9.310003041221439
				],
				[
					-18.80024148611321,
					-9.771658562008326
				],
				[
					-19.687300773944067,
					-10.233314082795214
				],
				[
					-20.574360061774925,
					-10.694969603582102
				],
				[
					-21.461419349605784,
					-11.15662512436899
				],
				[
					-22.34847863743664,
					-11.618280645155878
				],
				[
					-23.235537925267497,
					-12.079936165942765
				],
				[
					-24.122597213098352,
					-12.541591686729653
				],
				[
					-25.00965650092921,
					-13.00324720751654
				],
				[
					-25.896715788760066,
					-13.464902728303432
				],
				[
					-26.783775076590924,
					-13.92655824909032
				],
				[
					-27.67083436442178,
					-14.388213769877208
				],
				[
					-28.557893652252638,
					-14.849869290664095
				],
				[
					-29.444952940083496,
					-15.311524811450983
				],
				[
					-30.33201222791435,
					-15.773180332237871
				],
				[
					-31.21907151574521,
					-16.23483585302476
				],
				[
					-32.106130803576065,
					-16.696491373811646
				],
				[
					-32.99319009140692,
					-17.158146894598534
				],
				[
					-33.88024937923778,
					-17.619802415385422
				],
				[
					-34.764453059193265,
					-18.086026307058862
				],
				[
					-35.56806523295794,
					-18.681179626949802
				],
				[
					-36.37167740672262,
					-19.276332946840743
				],
				[
					-37.175289580487295,
					-19.871486266731683
				],
				[
					-37.97890175425198,
					-20.466639586622627
				],
				[
					-38.782513928016655,
					-21.061792906513567
				],
				[
					-39.58612610178133,
					-21.656946226404507
				],
				[
					-40.38973827554601,
					-22.252099546295447
				],
				[
					-41.193350449310685,
					-22.847252866186388
				],
				[
					-41.99696262307536,
					-23.44240618607733
				],
				[
					-42.800574796840046,
					-24.03755950596827
				],
				[
					-43.60418697060472,
					-24.632712825859212
				],
				[
					-44.4077991443694,
					-25.227866145750156
				],
				[
					-45.211411318134076,
					-25.823019465641096
				],
				[
					-46.01502349189876,
					-26.418172785532036
				],
				[
					-46.818635665663436,
					-27.013326105422976
				],
				[
					-47.62224783942811,
					-27.608479425313917
				],
				[
					-48.42586001319279,
					-28.20363274520486
				],
				[
					-49.229472186957466,
					-28.7987860650958
				],
				[
					-50.03308436072214,
					-29.393939384986744
				],
				[
					-50.836696534486826,
					-29.989092704877685
				],
				[
					-51.6403087082515,
					-30.584246024768625
				],
				[
					-52.44392088201618,
					-31.179399344659565
				],
				[
					-53.24753305578086,
					-31.774552664550505
				],
				[
					-54.05114522954554,
					-32.369705984441445
				],
				[
					-54.721227263943874,
					-33.110241831260495
				],
				[
					-55.38241082459142,
					-33.86046599717987
				],
				[
					-56.04359438523897,
					-34.61069016309926
				],
				[
					-56.70477794588652,
					-35.360914329018634
				],
				[
					-57.36596150653406,
					-36.11113849493802
				],
				[
					-58.027145067181614,
					-36.861362660857395
				],
				[
					-58.688328627829165,
					-37.61158682677677
				],
				[
					-59.34951218847671,
					-38.36181099269616
				],
				[
					-60.01069574912426,
					-39.112035158615534
				],
				[
					-60.6718793097718,
					-39.86225932453492
				],
				[
					-61.333062870419354,
					-40.612483490454295
				],
				[
					-61.994246431066905,
					-41.36270765637368
				],
				[
					-62.65542999171445,
					-42.112931822293056
				],
				[
					-63.316613552362,
					-42.86315598821244
				],
				[
					-63.977797113009544,
					-43.61338015413182
				],
				[
					-64.6389806736571,
					-44.363604320051195
				],
				[
					-65.30016423430465,
					-45.11382848597058
				],
				[
					-65.96134779495219,
					-45.864052651889956
				],
				[
					-66.62253135559973,
					-46.61427681780934
				],
				[
					-67.28371491624729,
					-47.36450098372872
				],
				[
					-67.94489847689483,
					-48.1147251496481
				],
				[
					-68.60608203754238,
					-48.86494931556748
				],
				[
					-69.26726559818994,
					-49.61517348148686
				],
				[
					-69.92844915883748,
					-50.36539764740624
				],
				[
					-70.58963271948502,
					-51.11562181332562
				],
				[
					-71.25081628013258,
					-51.865845979245
				],
				[
					-71.91199984078013,
					-52.616070145164386
				]
			]
                self.last_s = len(global_path)

                # # For Normal Arrive
                # last_idx_of_global = len(global_path)-1
                # x_increment = (
                #     global_path[last_idx_of_global][0]-global_path[last_idx_of_global-1][0])
                # y_increment = (
                #     global_path[last_idx_of_global][1]-global_path[last_idx_of_global-1][1])
                # while True:
                #     global_path.append(
                #         [(global_path[last_idx_of_global][0]+x_increment), (global_path[last_idx_of_global][1]+y_increment)])
                #     x_increment += x_increment
                #     y_increment += y_increment
                #     if x_increment >= 5 or y_increment >= 5:
                #         break

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
            # n_id = calc_idx(
            #     self.non_intp_path, (CS.position.x, CS.position.y))
            # now_lane_id = self.non_intp_id[n_id]
            now_lane_id = '0_0'
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
                #for visualize
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

                # if self.lmap.stoplines.get(now_lane_id) is not None:
                #     [trafficlight_x, trafficlight_y] = self.lmap.stoplines[now_lane_id][(len(self.lmap.stoplines[now_lane_id])+1)//2]
                #     stopline_idx = calc_idx(
                #     self.local_path, (trafficlight_x, trafficlight_y))
                #     print(idx, stopline_idx)

                #TODO: Avoidance Path
                #
                # if -1 < self.nearest_obstacle_distance and self.nearest_obstacle_distance <= 12.0 and len(self.lidar_obstacle) >= 0:
                #     if self.obstacle_detect_timer == 0.0:
                #         self.obstacle_detect_timer = time.time()
                #     if time.time()-self.obstacle_detect_timer >= 10:
                #         #print('[{}] 10sec have passed since an Obstacle was Detected'.format(self.__class__.__name__))
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

            # blinker = get_blinker(self.lmap.lanelets, self.non_intp_id, n_id)
            # self.pub_blinkiker.publish(blinker)

            pose = Pose()
            pose.position.x = 1
            pose.position.y = self.last_s  # m
            pose.position.z = s
            self.pub_goal_object.publish(pose)

            if self.last_s - s < 5.0:
                self.state = 'ARRIVED'
                #print('[{}] Arrived at Goal'.format(self.__class__.__name__))
            pp = 1

        elif self.state == 'ARRIVED':
            pp = 2

        return pp, self.local_path

#!/usr/bin/python

import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import copy
import time

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray, Int8
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
from jsk_recognition_msgs.msg import BoundingBoxArray

from lanelet_map import LaneletMap
from tile_map import TileMap
from micro_lanelet_graph import MicroLaneletGraph
from odom import Odom
from planner_utils import *
from viz import *


import kiapi_nodes
from kiapi_event import *
from fetch_objs import *
from new_velocity_plan import new_velocity_plan
from alter_path_planner import AlterPathPlanner


class Planner:
    def __init__(self):
        rospy.init_node('planner', anonymous=False)

        name = 'KIAPI'

        map_path = str(
            '/home/inha/catkin_ws/src/niro/planner/map/%s.json' % (name))
        tile_size = 5.0
        cut_dist = 10.0

        self.g_id = None
        self.g_idx = None

        self.lmap = LaneletMap(map_path)
        self.tmap = TileMap(self.lmap.lanelets, tile_size)
        self.graph = MicroLaneletGraph(self.lmap, cut_dist).graph
        self.alter_path = AlterPathPlanner()

        self.odom = Odom()
        self.car_v = 0.0
        self.traffic_lights = [0, 36001]
        self.lidar_object = []
        self.lidar_obstacle = []
        self.obstacle_marker_array = MarkerArray()

        self.temp_pt = None
        self.global_path = None
        self.non_intp_path = None
        self.non_intp_id = None

        self.local_update = True
        self.local_path = None
        self.local_point = None
        self.temp_global_idx = 0

        self.stage1_state = []
        self.stage1_mission = []
        self.arrive_info = Int16MultiArray()
        self.arrive_info.data = [0,0]

        self.parseNode = False
        self.test_mode = False
        self.stage1_nodes_phase1 = []
        self.stage1_nodes_phase2 = []
        self.object_markers = MarkerArray()
        self.bump_dist = 1000

        self.timer = None
        self.there_was_pedestrian = False

        self.pub_goal = rospy.Publisher(
            '/goal', Marker, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher(
            '/local_path', Marker, queue_size=1, latch=True)
        self.pub_final_path = rospy.Publisher(
            '/final_path', Marker, queue_size=1, latch=True)
        self.pub_target_v = rospy.Publisher(
            '/target_v', Float32, queue_size=1, latch=True)
        self.pub_current_laneID = rospy.Publisher(
            '/current_LaneID', Int16MultiArray, queue_size=1)
        self.pub_arrive_info = rospy.Publisher(
            '/arrive_info', Int16MultiArray, queue_size=10)
        self.pub_object_marker = rospy.Publisher(
            "/object_marker", MarkerArray, queue_size = 2)
        self.pub_blinkiker = rospy.Publisher(
            "/lane_change", Int8, queue_size = 2)

        rospy.Subscriber('/odom', Float32MultiArray, self.odom_cb)
        rospy.Subscriber('/car_v', Float32, self.ins_odom_cb)
        rospy.Subscriber('/spat_msg', Float32MultiArray, self.v2x_cb)
        rospy.Subscriber('/lidar/cluster_box', BoundingBoxArray, self.lidar_cluster_box_cb)
        rospy.Subscriber('/stage1_mission', PoseArray, self.stage1_mission_cb)
        rospy.Subscriber('/stage1_state', Int16MultiArray,
                         self.stage1_state_cb)
        rospy.Subscriber('/camera_ob_bump', Float32MultiArray, self.bump_dist_cb)
        self.alter = False

    def odom_cb(self, msg):
        self.odom.set(msg.data)

    def ins_odom_cb(self, msg):
        self.car_v = msg.data

    def v2x_cb(self, msg):
        self.traffic_lights = msg.data
    
    def bump_dist_cb(self, msg):
        self.bump_dist = msg.data[0]

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def object2frenet(self, obj_x, obj_y):  # Array in Frenet Array Out
        target = [obj_x, obj_y]
        point = self.local_point.query(target, 1)[1]
        if(point == 0):
            return 0, 1000
        wp = self.local_path

        n_x = wp[point][0] - wp[point-1][0]
        n_y = wp[point][1] - wp[point-1][1]
        x_x = target[0] - wp[point-1][0]
        x_y = target[1] - wp[point-1][1]

        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y

        frenet_d = self.distance(x_x, x_y, proj_x, proj_y)

        center_x = 900 - wp[point-1][0]
        center_y = -100 - wp[point-1][1]
        distToPos = self.distance(center_x, center_y, x_x, x_y)
        distToRef = self.distance(center_x, center_y, proj_x, proj_y)

        if(distToPos > distToRef):
            frenet_d *= -1

        return int(point/2), frenet_d

    def object2enu(self, obj_local_y, obj_local_x):
        rad = self.odom.yaw * (math.pi / 180.0)
        
        nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
        ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

        obj_x = self.odom.x + ny
        obj_y = self.odom.y + nx
        
        return obj_x, obj_y
    
    def make_object_marker(self, cw, x, y, idx):
        marker = Marker()
        
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3\
        marker.type = 2
        marker.id = idx
        
        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        if cw == 1:
            # Set the color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            # Set the color
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        
        # Set the pose of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker


    def lidar_cluster_box_cb(self, msg):
        objects = []
        for _, obj in enumerate(msg.boxes):
            x, y = obj.pose.position.x, obj.pose.position.y
            nx, ny = self.object2enu(x, y)
            objects.append([nx, ny])
        self.lidar_object = objects

    def stage1_mission_cb(self, msg):
        self.stage1_mission = msg.poses

    def stage1_state_cb(self, msg):
        self.stage1_state = msg.data

    def stage1ParseMission(self):
        rospy.loginfo("Parsing")
        for node in self.stage1_mission:
            node_info = {"type": int(node.orientation.x), "idx": int(
                node.orientation.y), "lat": float(node.orientation.z), "lng": float(node.orientation.w)}
            self.stage1_nodes_phase1.append(node_info)
            if(int(node.orientation.x) == 1):
                break
        for node in self.stage1_mission:
            if(int(node.orientation.x) == 1):
                continue
            node_info = {"type": int(node.orientation.x), "idx": int(
                node.orientation.y), "lat": float(node.orientation.z), "lng": float(node.orientation.w)}
            self.stage1_nodes_phase2.append(node_info)
            if(int(node.orientation.x) == 2):
                break
        if len(self.stage1_nodes_phase1) != 0 and len(self.stage1_nodes_phase2) != 0:
            self.parseNode = True

    def returnAppendedNonIntpPath(self, node_list):
        appended_non_intp_path = []
        appended_non_intp_id = []
        goal_pt = None
        for nodes in node_list:
            shortest_path = []
            goal_pt = convert2enu(
                self.lmap.basella, nodes["lat"], nodes["lng"])
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
                    self.g_id, self.g_idx = goal_lanelets
                    goal_viz = GoalViz(goal_pt)
                    self.pub_goal.publish(goal_viz)
                else:
                    rospy.logerr('Failed to match goal to lanelets!')
                    continue

                e_node = node_matching(self.lmap.lanelets, e_id, e_idx)
                g_node = node_matching(
                    self.lmap.lanelets, self.g_id, self.g_idx)

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

            appended_non_intp_path.extend(non_intp_path)
            appended_non_intp_id.extend(non_intp_id)

            self.temp_pt = goal_pt

        return appended_non_intp_path, appended_non_intp_id
    

    def run(self):
        rate = rospy.Rate(10)  # 10hz

        state = 'WAITING'
        passenger = None

        # velocity plan parameters
        min_v = 0.5  # km/h
        ref_v = 8.0  # km/h
        precision = 0.5

        st_param = {'s_min': -20.0, 's_max': 50.0,
                    't_max': 8.0, 'dt': 0.2, 'dt_exp': 1.0}
        
        lane_ID = Int16MultiArray()

        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlim([0.0, st_param['t_max']])
        ax.set_xticks([i for i in np.arange(0, int(st_param['t_max']), 0.5)])
        ax.grid(color='#BDBDBD', linestyle='-', linewidth=2, )
        drawn = None

        target_v = min_v
        l_idx = 0
        erase_global_path = []

        while not rospy.is_shutdown():
            self.pub_arrive_info.publish(self.arrive_info)
            self.pub_target_v.publish(Float32(target_v))
            if state == 'WAITING':
                rospy.loginfo("WAITING")
                if(len(self.stage1_state) != 0):
                    if self.stage1_state[0] == 0:
                        state = 'WAITING'
                        if not self.parseNode and not self.test_mode:
                            self.stage1ParseMission()
                    elif self.stage1_state[0] == 1 or self.stage1_state[0] == 4:
                        if(not self.stage1_state[1] and not self.stage1_state[2] and not self.stage1_state[3]):
                            rospy.loginfo('WAITING')
                            state = 'WAITING'
                            if not self.parseNode and not self.stage1_state[0] == 4 and not self.test_mode:
                                self.stage1ParseMission()
                        elif(self.arrive_info.data[0] == 0 and self.arrive_info.data[1] == 0 and self.stage1_state[1] and not self.stage1_state[2] and not self.stage1_state[3]):
                            rospy.loginfo('PHASE1')
                            if self.test_mode:
                                state = 'PHASE1'
                                passenger = 'Departure'
                            elif not self.parseNode and not self.stage1_state[0] == 4 and not self.test_mode:
                                self.stage1ParseMission()
                            if self.parseNode:
                                state = 'PHASE1'
                                passenger = 'Departure'
                        elif(self.arrive_info.data[1] == 0 and self.stage1_state[1] and self.stage1_state[2] and not self.stage1_state[3]):
                            rospy.loginfo('PHASE2')
                            if self.test_mode:
                                state = 'PHASE2'
                                passenger = 'Destination'
                            elif not self.parseNode and not self.stage1_state[0] == 4 and not self.test_mode:
                                self.stage1ParseMission()
                            if self.parseNode:
                                state = 'PHASE2'
                                passenger = 'Destination'
                        elif(self.stage1_state[1] and self.stage1_state[2] and self.stage1_state[3]):
                            rospy.loginfo('FINISH')
                            state = 'FINISH'
                    elif self.stage1_state[0] == 3:
                        state = 'FINISH'
                
            elif state == 'PHASE1' or state == 'PHASE2':
                appended_non_intp_path = None
                appended_non_intp_id = None
                node_list = None
                self.local_path = None
                self.temp_global_idx = 0
                l_idx = 0

                if state == 'PHASE1':
                    self.temp_pt = [self.odom.x, self.odom.y]
                    node_list = kiapi_nodes.STAGE1_NODES_PHASE1 if (
                        self.test_mode or self.stage1_state[0] == 4) else self.stage1_nodes_phase1
                elif state == 'PHASE2':
                    self.temp_pt = [self.odom.x, self.odom.y]
                    node_list = kiapi_nodes.STAGE1_NODES_PHASE2 if (
                        self.test_mode or self.stage1_state[0] == 4) else self.stage1_nodes_phase2

                if node_list is not None:
                    appended_non_intp_path, appended_non_intp_id = self.returnAppendedNonIntpPath(
                        node_list)

                if appended_non_intp_path is not None:
                    state = 'MOVE'
                    
                    global_path, max_v, last_s = ref_interpolate(
                        appended_non_intp_path, precision, min_v, ref_v)

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
                    self.non_intp_path = appended_non_intp_path
                    self.non_intp_id = appended_non_intp_id
                    erase_global_path = global_path
                    final_path_viz = FinalPathViz(self.global_path)
                    self.pub_final_path.publish(final_path_viz)

                else:
                    continue

            elif state == 'MOVE':
                _, idx = calc_cte_and_idx(
                    self.global_path, (self.odom.x, self.odom.y))
                
                if abs(idx-self.temp_global_idx) > 50:
                    continue
                else:
                    self.temp_global_idx = idx


                s = idx * precision

                _, n_id = calc_cte_and_idx(
                    self.non_intp_path, (self.odom.x, self.odom.y))
                split_now_id = (self.non_intp_id[n_id]).split('_')[0]

                if self.local_path is None or (self.local_path is not None and (len(self.local_path)-l_idx < 100) and len(self.local_path) > 100):

                    _, eg_idx = calc_cte_and_idx(
                        erase_global_path, (self.odom.x, self.odom.y))
                    local_path = []
                    if len(erase_global_path)-eg_idx > 500:
                        local_path = erase_global_path[eg_idx:eg_idx+500]
                    else:
                        local_path = erase_global_path[eg_idx:]
                    erase_global_path = erase_global_path[eg_idx:]

                    self.local_path = local_path
                    local_last_s = len(self.local_path)-1

                if self.local_path is not None:
                    _, l_idx = calc_cte_and_idx(
                        self.local_path, (self.odom.x, self.odom.y))
                    local_s = l_idx+2
                    self.local_point = KDTree(self.local_path)

                    local_path_viz = LocalPathViz(self.local_path)
                    if not self.alter:
                        self.pub_local_path.publish(local_path_viz)
                    else:
                        self.pub_local_path.publish(self.alter_path)

                    light_s = fetchStopLine(
                        self.non_intp_id[n_id]) if self.non_intp_id[n_id] is not None else 1000
                    light_state, light_time = self.traffic_lights

                    obstacle = []
                    pedestrian = []
                    markers = MarkerArray()
                    if len(self.lidar_object) > 0:
                        for _, obj in enumerate(self.lidar_object):
                            obj_s, obj_d = self.object2frenet(obj[0], obj[1])
                            if crosswalk_waypoint_matching2(self.tmap.tiles, self.tmap.tile_size, (obj[0], obj[1])):
                                pedestrian.append([1, obj_s, obj_d])
                                markers.markers.append(self.make_object_marker(1, obj[0], obj[1], _))
                            elif obj_s < 30 and obj_d > -1.5 and obj_d < 1.5:
                                obstacle.append([0, obj_s, obj_d])
                                markers.markers.append(self.make_object_marker(0, obj[0], obj[1], _))

                    self.lidar_obstacle = pedestrian + obstacle
                    self.object_markers = markers
                    self.pub_object_marker.publish(self.object_markers)

                    if self.bump_dist < 30:
                        obstacle.append([0, self.bump_dist + 14, 0])

                    speed_limit = (self.lmap.lanelets[split_now_id[0]
                                                      ]['speedLimit'])/3.6
                    local_max_v = ref_to_max_v(
                        self.local_path, precision, 1, min_v, ref_v, speed_limit)

                    #target_v, drawn = velocity_plan(ax, drawn, st_param, ref_v, local_last_s, local_s, local_max_v,
                    #                                self.odom.v, self.odom.a, light_s, light_state, light_time, self.lidar_obstacle)
                    # target_v = 3.0

                    for i in range(len(local_max_v)):
                        local_max_v[i]*=3.6
                
                    target_v, drawn = new_velocity_plan(ax, drawn, st_param, ref_v, local_last_s, local_s, local_max_v,
                                                    self.odom.v, self.odom.a, light_s, light_state, light_time, self.lidar_obstacle)
                    
                    if target_v < 0.1:
                        if len(pedestrian) > 0:
                            self.there_was_pedestrian = True
                        elif len(pedestrian) == 0 and self.there_was_pedestrian == True:
                            time.sleep(3)
                            self.there_was_pedestrian = False
                        
                        
                blinker = siginal_light_toggle(self.non_intp_path, n_id, precision, self.tmap, self.lmap, 1)
                self.pub_blinkiker.publish(blinker)
                lane_ID.data = [int(split_now_id)]
                self.pub_current_laneID.publish(lane_ID)

                fig.canvas.draw()
                fig.canvas.flush_events()

                if last_s - s < 8.0 and self.odom.v < 15.0/3.6:
                    state = 'ARRIVED'

            elif state == 'ARRIVED':
                if passenger == 'Departure':
                    rospy.loginfo("Departure")
                    self.arrive_info.data = [1, 0]
                    state = 'WAITING'
                elif passenger == 'Destination':
                    rospy.loginfo("Destination")
                    self.arrive_info.data = [1, 1]
                    state = 'FINISH'
                target_v = 0

            elif state == 'FINISH':
                state = 'FINISH'
                rospy.loginfo("FINISH")
                target_v = 0
                break


            rate.sleep()

        # While Over

if __name__ == "__main__":
    planner = Planner()
    planner.run()

#!/usr/bin/python
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray, Int8
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from jsk_recognition_msgs.msg import BoundingBoxArray

from config.config import Config
from libs.lanelet_map import LaneletMap
from libs.tile_map import TileMap
from micro_lanelet_graph import MicroLaneletGraph
from libs.odom import Odom
from planner_utils import *
from viz import *


class Planner:
    def __init__(self):
        rospy.init_node('planner', anonymous=False)

        self.config = Config()

        self.g_id = None
        self.g_idx = None

        self.lmap = LaneletMap(self.config.map_path)
        self.tmap = TileMap(self.lmap.lanelets, self.config.tile_size)
        self.graph = MicroLaneletGraph(self.lmap, self.config.cut_dist).graph

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

        self.get_goal = False

        self.object_markers = MarkerArray()

        self.pub_goal = rospy.Publisher(
            '/goal', Marker, queue_size=1, latch=True)
        self.pub_local_path = rospy.Publisher(
            '/local_path', Marker, queue_size=1, latch=True)
        self.pub_final_path = rospy.Publisher(
            '/final_path', Marker, queue_size=1, latch=True)
        self.pub_target_v = rospy.Publisher(
            '/target_v', Float32, queue_size=1, latch=True)
        self.pub_arrive_info = rospy.Publisher(
            '/arrive_info', Int16MultiArray, queue_size=10)
        self.pub_object_marker = rospy.Publisher(
            '/object_marker', MarkerArray, queue_size = 2)
        self.pub_blinkiker = rospy.Publisher(
            '/lane_change', Int8, queue_size = 2)
        

        rospy.Subscriber('/odom', Float32MultiArray, self.odom_cb)
        rospy.Subscriber('/car_v', Float32, self.ins_odom_cb)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/lidar/cluster_box', BoundingBoxArray, self.lidar_cluster_box_cb)
        self.alter = False

    def odom_cb(self, msg):
        self.odom.set(msg.data)

    def goal_cb(self, msg):
        self.goal_pt = [msg.pose.position.x, msg.pose.position.y]
        self.get_goal = True

    def ins_odom_cb(self, msg):
        self.car_v = msg.data

    def v2x_cb(self, msg):
        self.traffic_lights = msg.data
    
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
    

    def lidar_cluster_box_cb(self, msg):
        objects = []
        for _, obj in enumerate(msg.boxes):
            x, y = obj.pose.position.x, obj.pose.position.y
            nx, ny = self.object2enu(x, y)
            objects.append([nx, ny])
        self.lidar_object = objects

    def returnAppendedNonIntpPath(self, goal_pt):
        shortest_path = []
        
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

        return non_intp_path, non_intp_id
    

    def run(self):
        rate = rospy.Rate(10)  # 10hz

        state = 'WAITING'

        min_v = 0.0 
        ref_v = 8.0  
        precision = 0.5

        st_param = {'s_min': -20.0, 's_max': 50.0,
                    't_max': 8.0, 'dt': 0.2, 'dt_exp': 1.0}

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
                if self.get_goal:
                    state = 'READY'
                continue
                
            if state == 'READY':
                non_intp_path = None
                non_intp_id = None
                self.local_path = None
                self.temp_global_idx = 0
                l_idx = 0
                min_v = 0.5
                
                self.temp_pt = [self.odom.x, self.odom.y]

                non_intp_path, non_intp_id = self.returnAppendedNonIntpPath(
                    self.goal_pt)

                if non_intp_path is not None:
                    state = 'MOVE'
                    
                    global_path, max_v, last_s = ref_interpolate(
                        non_intp_path, precision, min_v, ref_v)

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
                    self.pub_local_path.publish(local_path_viz)

                    obstacle = []
                    markers = MarkerArray()
                    if len(self.lidar_object) > 0:
                        for _, obj in enumerate(self.lidar_object):
                            obj_s, obj_d = self.object2frenet(obj[0], obj[1])
                            if obj_s < 30 and obj_d > -1.5 and obj_d < 1.5:
                                obstacle.append([0, obj_s, obj_d])
                                markers.markers.append(make_object_marker(0, obj[0], obj[1], _))

                    self.lidar_obstacle = obstacle
                    self.object_markers = markers
                    self.pub_object_marker.publish(self.object_markers)

                    speed_limit = (self.lmap.lanelets[split_now_id[0]
                                                      ]['speedLimit'])/3.6
                    local_max_v = ref_to_max_v(
                        self.local_path, precision, 1, min_v, ref_v, speed_limit)

                    for i in range(len(local_max_v)):
                        local_max_v[i]*=3.6

                    light_s  = 15
                    light_time = 30
                    light_state = 0

                    target_v, drawn = new_velocity_plan(ax, drawn, st_param, ref_v, local_last_s, local_s, local_max_v,
                                                    self.odom.v, self.odom.a, light_s, light_time, light_state, self.lidar_obstacle)
                    

                blinker = signal_light_toggle(self.non_intp_path, n_id, precision, self.tmap, self.lmap, 1)
                self.pub_blinkiker.publish(blinker)

                fig.canvas.draw()
                fig.canvas.flush_events()

                if last_s - s < 8.0 and self.odom.v < 15.0/3.6:
                    state = 'ARRIVED'

            elif state == 'ARRIVED':
                rospy.loginfo('ARRIVED')
                target_v = 0
                if self.odom.v <= 0.1 :
                    break

            rate.sleep()

        # While Over

if __name__ == "__main__":
    planner = Planner()
    planner.run()

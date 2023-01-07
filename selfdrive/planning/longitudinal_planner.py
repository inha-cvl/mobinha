import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import heapq as hq
from shapely.geometry import Polygon
from shapely.geometry import Point as GPoint

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseArray

from selfdrive.planning.libs.map import LaneletMap
from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.viz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

class LongitudinalPlanner:
    def __init__(self, CP):
        self.lmap = LaneletMap(CP.mapParam.path)

        self.local_path = None
        self.lidar_obstacle = None
        self.now_lane_id = ''

        self.min_v = CP.minEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.target_v = 0.0
        self.st_param = CP.stParam._asdict()
        self.precision = CP.mapParam.precision

        plt.ion()
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_xlim([0.0, self.st_param["tMax"]])
        self.ax.set_xticks([i for i in np.arange(
            0, int(self.st_param["tMax"]), 0.5)])
        self.ax.grid(color='#BDBDBD', linestyle='-', linewidth=2, )
        self.drawn = None

        self.sub_local_path = rospy.Subscriber(
            '/mobinha/local_path', Marker, self.local_path_cb)
        self.sub_now_lane_id = rospy.Subscriber(
            '/now_lane_id', String, self.now_lane_id_cb
        )
        self.sub_lidar_obstacle = rospy.Subscriber(
            '/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)

        self.pub_target_v = rospy.Publisher(
            '/target_v', Float32, queue_size=1, latch=True)

    def local_path_cb(self, msg):
        self.local_path = [(pt.x, pt.y) for pt in msg.points]

    def now_lane_id_cb(self, msg):
        self.now_lane_id = msg.data

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z)
                               for pose in msg.poses]

    def object2enu(self, odom, obj_local_y, obj_local_x):
        rad = odom["yaw"] * (math.pi / 180.0)

        nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
        ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

        obj_x = odom["x"] + ny
        obj_y = odom["y"] + nx

        return obj_x, obj_y

    def obstacle_polygon(self, type, obs_time, obs_s, offset):
        polygon = [(0.0, obs_s-offset), (obs_time, obs_s-offset),
                   (obs_time, obs_s+offset), (0.0, obs_s+offset)]
        if type == 'last':
            color = 'gray'
        elif type == 'traffic_light':
            color = 'red'
        elif type == 'lidar_object':
            color = 'yellow'
        draw = self.ax.add_patch(patches.Polygon(
            [pt for pt in polygon], closed=True, edgecolor='black', facecolor=color))
        self.drawn.append(draw)
        return polygon

    def velocity_plan(self, ref_v, last_s, s, max_v, cur_v, traffic_lights, lidar_objects):
        # cur_v : m/s
        # s = 0.5m
        ref_v *= KPH_TO_MPS
        s_min, s_max, t_max = self.st_param['sMin'], self.st_param['sMax'], self.st_param['tMax']
        dt, dt_exp = self.st_param['dt'], self.st_param['dtExp']

        target_v = 0.0
        s += 1.0

        if self.drawn is not None:
            for d in self.drawn:
                d.remove()
        self.drawn = []
        self.ax.set_ylim([s+s_min, s+s_max])
        self.ax.set_yticks([i for i in range(int(s+s_min), int(s+s_max))])

        ############## Consider Obstacles ###########################
        obstacles = []

        # Maintain Distance to Goal
        last = self.obstacle_polygon('last', t_max, last_s, 3)
        obstacles.append(last)

        # Traffic Light
        light_time = min(t_max-3.0, traffic_lights[1]/10)
        # traffic_light = self.obstacle_polygon('traffic_light', light_time,traffic_lights[0],10)
        # obstacles.append(traffic_light)

        # LiDAR Objects
        obj_time = 3 + cur_v*MPS_TO_KPH
        if lidar_objects is not None:
            for os in lidar_objects:
                if os[2] >= -1.0 and os[2] <= 1.0:
                    lidar_object = self.obstacle_polygon(
                        'lidar_object', obj_time, os[1], 20)
                    obstacles.append(lidar_object)

        ############### Search-Based Optimal Velocity Planning  ( A* )###############
        # https://arxiv.org/pdf/1803.04868.pdf
        start = (0.0, s, cur_v)  # t, s, v
        open_heap = []
        open_dict = {}
        visited_dict = {}

        def goal_cost(node):
            return last_s - node[1]

        # hq : Min-Heap, parent < child
        # put a tuple in the heap, it is composed of the first element.
        hq.heappush(open_heap, (0 + goal_cost(start), start))
        open_dict[start] = (0 + goal_cost(start), start, start)

        while len(open_heap) > 0:
            d_node = open_heap[0][1]  # (t,s,v), parent
            node_total_cost = open_heap[0][0]  # last_s - s
            visited_dict[d_node] = open_dict[d_node]

            # get final trajectory
            # time is bigger than 8 sec or s(longitudinal distance) is bigger than 50
            if d_node[0] > t_max or d_node[1] > s + s_max:
                final_path = []
                node = d_node  # parent

                while True:
                    open_node_contents = visited_dict[node]
                    node = open_node_contents[2]
                    final_path.append(node)
                    if node == start:
                        break

                # to make large cost -> small cost
                final_path.reverse()

                t_list = []
                s_list = []
                v_list = []
                for pt in final_path:
                    t_list.append(pt[0])
                    s_list.append(pt[1])
                    v_list.append(pt[2])

                target_v = v_list[1]  # for safety
                d = self.ax.plot(t_list, s_list, '-m')
                self.drawn.append(d[0])

                break

            # pop
            hq.heappop(open_heap)

            # push
            for a in [-3.0, -1.5, 0.0, 1.0]:
                t_exp = d_node[0]
                s_exp = d_node[1]
                v_exp = d_node[2]

                skip = False
                for _ in range(int(dt_exp // dt + 1)):  # range(5), dtExp= 1.0, dt = 0.2
                    if not skip:
                        t_exp = round((t_exp+dt),1)
                        s_exp += v_exp * dt
                        v_exp += a * dt

                        idx = int((s_exp - (s_exp % 0.5)) / 0.5)
                        v_max = max_v[idx] if idx < len(
                            max_v) else ref_v*KPH_TO_MPS

                        if v_exp < 0.0 or v_exp > v_max:
                            skip = True
                            break

                        point = GPoint(t_exp, s_exp)
                        for obstacle in obstacles:
                            if point.within(Polygon(obstacle)):
                                skip = True
                                break
                if skip:
                    continue

                neighbor = (t_exp, s_exp, v_exp) 
                cost_to_neighbor = node_total_cost - goal_cost(d_node)
                heurestic = goal_cost((t_exp, s_exp, v_exp))
                total_cost = heurestic + cost_to_neighbor

                skip = False
                found_lower_cost_path_in_open = False

                if neighbor in open_dict:
                    if total_cost > open_dict[neighbor][0]:
                        skip = True
                    elif neighbor in visited_dict:
                        if total_cost > visited_dict[neighbor][0]:
                            found_lower_cost_path_in_open = True

                # if new node is better than parent
                if skip == False and found_lower_cost_path_in_open == False:
                    hq.heappush(open_heap, (total_cost, neighbor))
                    open_dict[neighbor] = (total_cost, neighbor, d_node)

        return target_v

    def run(self, sm, pp=0):
        CS = sm.CS
        lgp = 0

        self.pub_target_v.publish(Float32(self.target_v))

        if self.local_path is not None:

            _, l_idx = calc_cte_and_idx(
                self.local_path, (CS.position.x, CS.position.y))

            split_now_lane_id = self.now_lane_id.split('_')[0]
            speed_limit = (
                self.lmap.lanelets[split_now_lane_id]['speedLimit'])

            local_max_v = ref_to_max_v(
                self.local_path, self.precision, 1, self.min_v, self.ref_v, speed_limit)

            tl_objects = [15, 30, 2]  # s, time, state

            self.target_v = self.velocity_plan(self.ref_v, len(
                self.local_path), l_idx, local_max_v, CS.vEgo, tl_objects, self.lidar_obstacle)

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.0001:
                    lgp = 2
            elif pp == 4:
                self.target_v = 0.0
            else:
                lgp = 1

        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()

        return lgp

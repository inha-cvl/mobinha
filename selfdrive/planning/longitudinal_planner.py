import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import heapq as hq
from shapely.geometry import Polygon
from shapely.geometry import Point as GPoint

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseArray

from libs.map import LaneletMap
from libs.planner_utils import *
from selfdrive.visualize.viz_utils import *


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
            '/local_path', Marker, self.local_path_cb)
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

    def velocity_plan(self, ref_v, last_s, s, max_v, cur_v, cur_a, tl_objects, objects_s):
        # cur_v : m/s
        ref_v *= KPH_TO_MPS
        s_min, s_max, t_max = self.st_param['sMin'], self.st_param['sMax'], self.st_param['tMax']
        dt, dt_exp = self.st_param['dt'], self.st_param['dtExp']
        tl_s, tl_time, tl_state = tl_objects

        target_v = 0.0
        s += 1.0

        if self.drawn is not None:
            for d in self.drawn:
                d.remove()

        self.drawn = []
        obstacles = []

        last = [(0.0, last_s-1.0), (t_max, last_s-1.0),
                (t_max, last_s+5.0), (0.0, last_s+5.0)]
        obstacles.append(last)

        tl_offset = 10

        light_time = min(t_max-3.0, tl_time/10)
        traffic_light = [(0, tl_s), (light_time, tl_s),
                        (light_time, tl_s+tl_offset), (0, tl_s+tl_offset)]

        obj_offset = 5
        if cur_v < 30 * KPH_TO_MPS:
            obj_time = 3
        elif cur_v < 40 * KPH_TO_MPS:
            obj_time = 5
        elif cur_v < 100 * KPH_TO_MPS:
            obj_time = 7
        if objects_s is not None:
            for os in objects_s:
                if os[2] > -1.5 and os[2] < 1.5:
                    obj = [(0.0, os[1]-obj_offset), (obj_time, os[1]-obj_offset),
                        (obj_time, os[1]+obj_offset), (0.0, os[1]+obj_offset)]

                    obstacles.append(obj)
                    obstacle = self.ax.add_patch(patches.Polygon(
                        [pt for pt in obj], closed=True, edgecolor='black', facecolor='yellow'))
                    self.drawn.append(obstacle)

        last = self.ax.add_patch(patches.Polygon(
            [pt for pt in last], closed=True, edgecolor='black', facecolor='gray'))
        self.drawn.append(last)

        self.ax.set_ylim([s+s_min, s+s_max])
        self.ax.set_yticks([i for i in range(int(s+s_min), int(s+s_max))])

        ############### Planning Start ###############
        start = (0.0, s, cur_v, cur_a)  # t, s, v, a
        open_heap = []
        open_dict = {}
        visited_dict = {}

        def goal_cost(node):
            return last_s - node[1]

        hq.heappush(open_heap, (0 + goal_cost(start), start))
        open_dict[start] = (0 + goal_cost(start), start, (start, start))

        while len(open_heap) > 0:
            chosen_d_node = open_heap[0][1]
            chosen_node_total_cost = open_heap[0][0]
            chosen_c_node = open_dict[chosen_d_node][1]
            visited_dict[chosen_d_node] = open_dict[chosen_d_node]

            if chosen_d_node[0] > t_max or chosen_d_node[1] > s + s_max:
                final_path = []
                node = chosen_d_node

                while True:
                    open_node_contents = visited_dict[node]
                    parent_of_node = open_node_contents[2][1]
                    final_path.append(parent_of_node)
                    node = open_node_contents[2][0]

                    if node == start:
                        break

                final_path.reverse()

                t_list = []
                s_list = []
                v_list = []
                for pt in final_path:
                    t_list.append(pt[0])
                    s_list.append(pt[1])
                    v_list.append(pt[2])

                target_v = v_list[1]

                d = self.ax.plot(t_list, s_list, '-m')
                self.drawn.append(d[0])

                break

            hq.heappop(open_heap)

            for a in [-3.0, -1.5, 0.0, 1.0]:
                t_exp = chosen_c_node[0]
                s_exp = chosen_c_node[1]
                v_exp = chosen_c_node[2]
                a_exp = chosen_c_node[3]

                skip = False
                for _ in range(int(dt_exp // dt + 1)):
                    if not skip:
                        t_exp += dt
                        t_exp = round(t_exp, 1)
                        s_exp += v_exp * dt
                        v_exp += a * dt

                        idx = int((s_exp - (s_exp % 0.5)) / 0.5)

                        if idx < len(max_v):
                            v_max = max_v[idx]
                        else:
                            v_max = 300 * KPH_TO_MPS

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

                neighbor_t = t_exp
                neighbor_s = s_exp
                neighbor_v = v_exp
                neighbor_a = a

                d = self.ax.plot([chosen_c_node[0], neighbor_t], [
                    chosen_c_node[1], neighbor_s], '-c')
                self.drawn.append(d[0])

                neighbor_t_d = neighbor_t
                neighbor_s_d = neighbor_s

                cost_to_neighbor_from_start = chosen_node_total_cost - \
                    goal_cost(chosen_d_node)

                neighbor = ((neighbor_t_d, neighbor_s_d, neighbor_v, neighbor_a),
                            (neighbor_t, neighbor_s, neighbor_v, neighbor_a))

                heurestic = goal_cost(
                    (neighbor_t_d, neighbor_s_d, neighbor_v, neighbor_a))

                cost_to_neighbor_from_start = 0.1 * \
                    (a - a_exp)**2 + 0.1 * t_exp + (ref_v - v_exp)**2

                total_cost = heurestic + cost_to_neighbor_from_start

                skip = 0
                found_lower_cost_path_in_open = 0

                if neighbor[0] in open_dict:
                    if total_cost > open_dict[neighbor[0]][0]:
                        skip = 1

                    elif neighbor[0] in visited_dict:
                        if total_cost > visited_dict[neighbor[0]][0]:
                            found_lower_cost_path_in_open = 1

                if skip == 0 and found_lower_cost_path_in_open == 0:
                    hq.heappush(open_heap, (total_cost, neighbor[0]))
                    open_dict[neighbor[0]] = (
                        total_cost, neighbor[1], (chosen_d_node, chosen_c_node))

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
                self.local_path), l_idx, local_max_v, CS.vEgo, CS.aEgo, tl_objects, self.lidar_obstacle)

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

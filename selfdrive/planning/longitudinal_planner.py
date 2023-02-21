import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import heapq as hq
from shapely.geometry import Polygon
from shapely.geometry import Point as GPoint

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseArray, Pose

from selfdrive.planning.libs.map import LaneletMap
from selfdrive.planning.libs.planner_utils import *
from selfdrive.visualize.viz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
IDX_TO_M = 0.5
M_TO_IDX = 2

VIZ_GRAPH = True


class LongitudinalPlanner:
    def __init__(self, CP):

        self.lidar_obstacle = None
        self.traffic_light_obstacle = None
        self.lane_information = None
        self.goal_object = None

        self.min_v = CP.minEnableSpeed
        self.ref_v = CP.maxEnableSpeed
        self.wheel_base = CP.wheelbase
        self.target_v = self.min_v*KPH_TO_MPS
        self.st_param = CP.stParam._asdict()
        self.cc_param = CP.ccParam._asdict()
        self.precision = CP.mapParam.precision

        if VIZ_GRAPH:
            plt.ion()
            self.fig = plt.figure(figsize=(10, 10))
            self.ax = self.fig.add_subplot(1, 1, 1)
            self.ax.set_xlim([0.0, self.st_param["tMax"]])
            self.ax.set_xticks([i for i in np.arange(
                0, int(self.st_param["tMax"]), 0.5)])
            self.ax.grid(color='#BDBDBD', linestyle='-', linewidth=2, )
            self.drawn = None

        rospy.Subscriber(
            '/mobinha/perception/lidar_obstacle', PoseArray, self.lidar_obstacle_cb)
        rospy.Subscriber('/mobinha/perception/traffic_light_obstacle',
                         PoseArray, self.traffic_light_obstacle_cb)
        rospy.Subscriber('/mobinha/planning/lane_information',
                         Pose, self.lane_information_cb)
        rospy.Subscriber(
            '/mobinha/planning/goal_information', Pose, self.goal_object_cb)
        self.pub_target_v = rospy.Publisher(
            '/mobinha/planning/target_v', Float32, queue_size=1, latch=True)
        self.pub_trajectory = rospy.Publisher(
            '/mobinha/planning/trajectory', PoseArray, queue_size=1)

    def lidar_obstacle_cb(self, msg):
        self.lidar_obstacle = [(pose.position.x, pose.position.y, pose.position.z)
                               for pose in msg.poses]

    def traffic_light_obstacle_cb(self, msg):
        self.traffic_light_obstacle = [
            (pose.position.x, pose.position.y, pose.position.z) for pose in msg.poses]

    def lane_information_cb(self, msg):
        # id, forward_direction, cross_walk distance
        self.lane_information = [msg.position.x,
                                 msg.position.y, msg.position.z]

    def goal_object_cb(self, msg):
        self.goal_object = (msg.position.x, msg.position.y, msg.position.z)

    def obstacle_polygon(self, obj, s, cur_v):
        # [0] Dynamic [1] Static [2] Traffic Light
        i = int(obj[0])
        color = ['yellow', 'gray', 'red']
        obs_time = [5, self.st_param['tMax'], 7]  # sec
        offset = [6+(cur_v*obs_time[i]), 5+(cur_v*MPS_TO_KPH), 15]  # m
        offset = [os*M_TO_IDX for os in offset]
        pos = obj[1] + s if obj[0] == 1 else obj[1]

        polygon = [(0.0, pos-2*offset[i]), (obs_time[i], pos-2*offset[i]),
                   (obs_time[i], pos), (0.0, pos)]

        if VIZ_GRAPH:
            draw = self.ax.add_patch(patches.Polygon(
                [pt for pt in polygon], closed=True, edgecolor='black', facecolor=color[i]))
            self.drawn.append(draw)
        return polygon

    def velocity_plan(self, cur_v, ref_v, max_v, last_s, s, object_list):
        # cur_v : m/s
        # s = 0.5m
        s_min, s_max, t_max = self.st_param['sMin'], self.st_param['sMax'], self.st_param['tMax']
        dt, dt_exp = self.st_param['dt'], self.st_param['dtExp']
        target_v = ref_v
        s += 3.0  # loof a little b it forward

        if VIZ_GRAPH:
            if self.drawn is not None:
                for d in self.drawn:
                    d.remove()
            self.drawn = []
            self.ax.set_ylim([s+s_min, s+s_max])
            self.ax.set_yticks([i for i in range(int(s+s_min), int(s+s_max))])

        obstacles = []
        for obj in object_list:
            obs = self.obstacle_polygon(obj, s, cur_v)
            obstacles.append(obs)

        ############### Search-Based Optimal Velocity Planning  ( A* )###############
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

        test_i = 0
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
                target_v = final_path[-1][2]  # for safety
                if VIZ_GRAPH:
                    t_list = []
                    s_list = []
                    for pt in final_path:
                        t_list.append(pt[0])
                        s_list.append(pt[1])

                    d = self.ax.plot(t_list, s_list, '-m')
                    self.drawn.append(d[0])

                break

            # pop
            hq.heappop(open_heap)

            # push

            for a in [-10.0, -3.0, -1.5, 0.0, 1.5]:
                t_exp, s_exp, v_exp = d_node

                skip = False
                for _ in range(int(dt_exp // dt + 1)):  # range(5), dtExp= 1.0, dt = 0.2
                    t_exp = round(t_exp+dt, 1)
                    s_exp += (v_exp * dt) * M_TO_IDX
                    v_exp += a * dt

                    # if v_exp > max_v:
                    #     skip = True
                    #     break

                    point = GPoint(t_exp, s_exp)
                    for obstacle in obstacles:
                        if point.within(Polygon(obstacle)):
                            skip = True
                            break

                # if skip:
                #     continue

                neighbor = (t_exp, s_exp, v_exp)
                cost_to_neighbor = node_total_cost - goal_cost(d_node)
                heurestic = goal_cost(neighbor)
                total_cost = heurestic + cost_to_neighbor

                # not_low_cost = False
                # found_lower_cost_path_in_open = False

                # if neighbor in open_dict:
                #     if total_cost > open_dict[neighbor][0]:
                #         not_low_cost = True
                #     elif neighbor in visited_dict:
                #         if total_cost > visited_dict[neighbor][0]:
                #             found_lower_cost_path_in_open = True

                # if not_low_cost == False and found_lower_cost_path_in_open == False:
                hq.heappush(open_heap, (total_cost, neighbor))
                open_dict[neighbor] = (total_cost, neighbor, d_node)

                if skip:
                    break

            test_i += 1
        
        target_v = max_v if target_v > max_v else target_v
        return target_v

    def traffic_light_to_obstacle(self, traffic_light, forward_direction):
        # straight, turn_left, turn_right, left_change, right_change, u-turn
        # TODO: consideration filtering
        consideration_class_list = [[6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13], [
            6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [6, 8, 10, 11, 12, 13], [4, 6, 8, 9, 10, 11, 13]]
        if traffic_light in consideration_class_list[forward_direction]:
            return True
        else:
            return False

    def check_objects(self, local_len):
        object_list = []

        # [0] = Dynamic Object
        if self.lidar_obstacle is not None:
            for lobs in self.lidar_obstacle:
                if lobs[2] >= -2.0 and lobs[2] <= 2.0:  # object in my lane
                    object_list.append(lobs)

        # [1] = Goal Object
        if self.goal_object is not None:
            left = (self.goal_object[1]-self.goal_object[2]) * M_TO_IDX
            if left <= local_len:
                object_list.append(
                    [self.goal_object[0], left, 0])

        # [2] = Traffic Light
        if self.traffic_light_obstacle is not None:
            for tlobs in self.traffic_light_obstacle:
                if self.traffic_light_to_obstacle(int(tlobs[1]), int(self.lane_information[1])):
                    object_list.append((tlobs[0], self.lane_information[2], 0))
        return object_list

    def run(self, sm, pp=0, local_path=None):
        CS = sm.CS
        lgp = 0
        self.pub_target_v.publish(Float32(self.target_v))

        if local_path is not None:
            l_idx = calc_idx(
                local_path, (CS.position.x, CS.position.y))
            local_max_v, curvature, tx, ty = max_v_by_curvature(
                local_path, l_idx, self.ref_v, CS.yawRate)
            object_list = self.check_objects(len(local_path))
            self.target_v = self.velocity_plan(
                CS.vEgo, self.target_v, local_max_v, len(local_path), l_idx, object_list)

            if pp == 2:
                self.target_v = 0.0
                if CS.vEgo <= 0.0001:
                    lgp = 2
            elif pp == 4:
                self.target_v = 0.0
            else:
                lgp = 1
                trajectory = PoseArray()
                for i, x in enumerate(tx):
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = ty[i]
                    pose.position.z = curvature
                    trajectory.poses.append(pose)
                self.pub_trajectory.publish(trajectory)

        if VIZ_GRAPH:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        return lgp

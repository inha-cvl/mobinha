import math

import heapq as hq
import numpy as np
import pymap3d as pm
from scipy.spatial import KDTree

import libs.cubic_spline_planner as cubic_spline_planner
from libs.quadratic_spline_interpolate import QuadraticSplineInterpolate
from selfdrive.visualize.viz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
IDX_TO_M = 0.5
M_TO_IDX = 2


def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)


def convert2enu(base, lat, lng):
    x, y, _ = pm.geodetic2enu(lat, lng, 20, base[0], base[1], base[2])
    return [x, y]


def lanelet_matching(tile, tile_size, t_pt):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):

            selected_tile = tile.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            min_dist = dist
                            l_id = id_
                            l_idx = data['idx'][idx]

    if l_id is not None:
        return (l_id, l_idx)
    else:
        return None


def exchange_waypoint(target, now):
    exchange_waypoints = []

    for n_pt in now:
        min_dist = float('inf')
        change_pt = []
        for t_pt in target:
            dist = euc_distance(t_pt, n_pt)
            if dist < min_dist:
                min_dist = dist
                change_pt = t_pt
        exchange_waypoints.append(change_pt)

    return exchange_waypoints


def generate_avoid_path(lanelets, now_lane_id, local_path_from_now, obs_len):

    left_id = lanelets[now_lane_id]['adjacentLeft']
    right_id = lanelets[now_lane_id]['adjacentRight']

    if left_id is not None or right_id is not None:
        obs_idx = obs_len*M_TO_IDX if obs_len * \
            M_TO_IDX < len(local_path_from_now) else len(local_path_from_now)-1
        local_path_from_now = local_path_from_now[:obs_idx+1]

    else:
        return None

    avoid_path = None
    if left_id is not None:
        # print(lanelets[left_id])
        avoid_path = exchange_waypoint(
            lanelets[left_id]['waypoints'], local_path_from_now)

    if right_id is not None:
        avoid_path = exchange_waypoint(
            lanelets[right_id]['waypoints'], local_path_from_now)

    return avoid_path


def get_nearest_crosswalk(lanelets, now_lane_id, local_point):
    crosswalk = []
    for id_, data in lanelets.items():
        if id_ == now_lane_id:
            if len(data['crosswalk']) > 0:
                crosswalks = data['crosswalk']
                for arr in crosswalks:
                    crosswalk.extend(arr)
    now_cw_idx = math.inf

    for cw_wp in crosswalk:
        idx = local_point.query(cw_wp, 1)[1]
        if idx < now_cw_idx:
            now_cw_idx = idx

    return now_cw_idx


def interpolate(points, precision):
    def filter_same_points(points):
        filtered_points = []
        pre_pt = None

        for pt in points:
            if pre_pt is None or pt != pre_pt:
                filtered_points.append(pt)

            pre_pt = pt

        return filtered_points

    points = filter_same_points(points)
    if len(points) < 2:
        return points, None, None, None

    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))

    itp_points = []
    s = []
    yaw = []
    k = []

    for n, ds in enumerate(np.arange(0.0, itp.s[-1], precision)):
        s.append(ds)
        x, y = itp.calc_position(ds)
        dyaw = itp.calc_yaw(ds)

        dk = itp.calc_curvature(ds)

        itp_points.append((float(x), float(y)))
        yaw.append(dyaw)
        k.append(dk)

    return itp_points, s, yaw, k


def ref_interpolate(points, precision, min_v, ref_v):
    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))
    itp_points = []
    for n, ds in enumerate(np.arange(0.0, itp.s[-1], precision)):
        x, y = itp.calc_position(ds)
        itp_points.append((float(x), float(y)))

    return itp_points, itp.s[-1]


def node_matching(lanelet, l_id, l_idx):
    node_id = l_id

    if lanelet[l_id].get('cut_idx') is not None:
        for n, (s_idx, e_idx) in enumerate(lanelet[l_id]['cut_idx']):
            if l_idx >= s_idx and l_idx < e_idx:
                node_id += '_%s' % (n)
                break

    return node_id


def dijkstra(graph, start, finish):
    distances = {}
    previous = {}
    nodes = []

    for vertex in graph:
        if vertex == start:
            distances[vertex] = 0
            hq.heappush(nodes, [distances[vertex], vertex])
        else:
            distances[vertex] = float('inf')
            hq.heappush(nodes, [distances[vertex], vertex])
        previous[vertex] = None

    while nodes:
        current = hq.heappop(nodes)[1]

        if current == finish:
            path = []
            if previous[current] is not None:
                while previous[current]:
                    path.append(current)
                    current = previous[current]
                path.append(start)
                path.reverse()
                cost = distances[finish]
                return (path, cost)

            else:
                return None

        neighbors = graph[current]

        for neighbor in neighbors:
            if neighbor == start:
                continue
            # cost(start->current) + cost(current->neighbor)
            bridge_cost = distances[current] + neighbors[neighbor]

            # found shortest path! -> update!
            if bridge_cost < distances[neighbor]:
                distances[neighbor] = bridge_cost
                previous[neighbor] = current

                for node in nodes:
                    if node[1] == neighbor:
                        node[0] = bridge_cost
                        break
                hq.heapify(nodes)  # heapq relocate

    return None


def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points


def node_to_waypoints2(lanelet, shortest_path):
    final_path = []
    final_id_path = []

    for id in shortest_path:
        alpha_path = []
        split_id = id.split('_')
        if len(split_id) == 2:
            s_idx, e_idx = (lanelet[split_id[0]]['cut_idx'][int(split_id[1])])
            alpha_path.append(lanelet[split_id[0]]
                              ['waypoints'][int((s_idx+e_idx)//2)])
            for i in range(len(alpha_path)):
                final_id_path.append(str("{}{}".format(id, i)))
            final_path.extend(alpha_path)

        else:
            alpha_path.extend(lanelet[split_id[0]]['waypoints'])
            for i in range(len(alpha_path)):
                final_id_path.append(str("{}_{}".format(id, i)))
            final_path.extend(alpha_path)
    return final_path, final_id_path


def get_direction_number(lanelet, splited_id, forward_direction):  # lanlet, 0, 'S'
    direction_list = lanelet[splited_id]["direction"]
    direction_dir = {'S': 0, 'L': 1, 'R': 2, 'U': 3}
    direction_number = 0
    if len(direction_list) == 1:
        direction_number = direction_dir[direction_list[0]]
    elif len(direction_list) > 1:
        if forward_direction in direction_list:
            direction_number = direction_dir[forward_direction]
    else:
        direction_number = direction_dir[forward_direction]
    return direction_number


def find_nearest_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx


def calc_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    if min_idx == len(pts) - 1:
        pt1 = pts[min_idx-1]
        pt2 = pts[min_idx]
    else:
        pt1 = pts[min_idx]
        pt2 = pts[min_idx+1]

    return min_idx


def ref_to_csp(ref_path):
    x_list, y_list = zip(*ref_path)
    csp = cubic_spline_planner.Spline2D(x_list, y_list)
    return csp


def max_v_by_curvature(path, i, ref_v, cur_v, yawRate, ws=80, curv_threshold=200):
    i -= 10 if i > 10 else 0
    return_v = ref_v
    x = []
    y = []
    curvated = 1000
    #ws = ws + int(20*cur_v)

    if i < len(path)-1:
        if i+ws < len(path):
            x = [v[0] for v in path[i:i+ws]]
            y = [v[1] for v in path[i:i+ws]]
        else:
            x = [v[0] for v in path[i:]]
            y = [v[1] for v in path[i:]]

        x = np.array([(v-x[0]) for v in x])
        y = np.array([(v-y[0]) for v in y])

        origin_plot = np.vstack((x, y))
        rotation_radians = math.radians(-yawRate) + math.pi/2
        rotation_mat = np.array([[math.cos(rotation_radians), -math.sin(rotation_radians)],
                                 [math.sin(rotation_radians), math.cos(rotation_radians)]])
        rotation_plot = rotation_mat@origin_plot
        x, y = rotation_plot

        if len(x) > 2:
            cr = np.polyfit(x, y, 2)
            if cr[0] != 0:
                curvated = ((1+(2*cr[0]+cr[1])**2) ** 1.5)/np.absolute(2*cr[0])
            else:
                curvated = curv_threshold
        else:
            curvated = curv_threshold+1

        if curvated < curv_threshold:
            return_v = ref_v - (abs(curv_threshold-curvated)*0.5)
            return_v = return_v if return_v > 0 else 7
    curvated = 1000 if curvated > 1000 else curvated
    return return_v*KPH_TO_MPS, curvated, x, y


def get_forward_direction(lanelets, next_id):  # (global_path, i, ws=200):
    # return direction - 0:straight, 1:left, 2:right,3:left lane change, 4:right lane change, 5:U-turn
    link = lanelets[next_id]
    p = (link['waypoints'][0][0], link['waypoints'][0][1])
    q = (link['waypoints'][len(link['waypoints'])//2][0],
         link['waypoints'][len(link['waypoints'])//2][1])
    r = (link['waypoints'][-1][0], link['waypoints'][-1][1])

    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])

    if link['uTurn']:
        return 'U'
    if link['rightTurn']:
        return 'R'
    if link['leftTurn']:
        return 'L'
    threshold = 100
    if -threshold < val < threshold:
        return 'S'  # collinear
    elif val <= -threshold:
        return 'L'  # left turn
    elif val >= threshold:
        return 'R'  # right turn


def get_blinker(lanelets, ids, ego_idx, look_forward=40):
    look_forward *= M_TO_IDX
    blinker = 0

    now_id = ids[ego_idx].split('_')[0]

    left_id = lanelets[now_id]['adjacentLeft']
    right_id = lanelets[now_id]['adjacentRight']

    try:
        forward_ids = np.array([fid.split('_')[0]
                               for fid in ids[ego_idx:ego_idx+look_forward]])
        if np.any(forward_ids == left_id):
            blinker = 1
        elif np.any(forward_ids == right_id):
            blinker = 2
        else:
            blinker = 0
        return blinker
    except IndexError:
        return blinker


def set_lane_ids(lst):
    lst = [elem.split("_")[0] for elem in lst]

    lane_ids = [lst[0]]

    for i in range(1, len(lst)):
        if lst[i] != lst[i-1]:
            lane_ids.append(lst[i])

    return lane_ids

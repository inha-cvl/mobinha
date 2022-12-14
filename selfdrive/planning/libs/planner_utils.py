import math

import heapq as hq
import numpy as np
import pymap3d as pm

import libs.cubic_spline_planner as cubic_spline_planner
from libs.quadratic_spline_interpolate import QuadraticSplineInterpolate

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6


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


IDX_TO_M = 0.5
M_TO_IDX = 2


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


def lanelet_matching_adj(tile, tile_size, t_pt):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    left_id, right_id = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tile.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            min_dist = dist
                            left_id = data['adjacentLeft']
                            right_id = data['adjacentRight']

    return (left_id, right_id)


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
    # points = filter_same_points(points)

    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))

    itp_points = []
    max_v = []

    for n, ds in enumerate(np.arange(0.0, itp.s[-1], precision)):
        x, y = itp.calc_position(ds)
        itp_points.append((float(x), float(y)))
        dk = itp.calc_curvature(ds)

        if dk != 0.0 and dk is not None:
            R = abs(1.0 / dk)
            curvature_v = math.sqrt(127 * 0.25 * R)
        else:
            curvature_v = 300

        v = max(min_v, min(curvature_v, ref_v))
        max_v.append(v * KPH_TO_MPS)

    # return itp_points, max_v, itp.s[-1], itp
    return itp_points, max_v, itp.s[-1]


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


def find_nearest_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx


def cross_track_error(pt1, pt2, pt):
    x1, y1 = pt1
    x2, y2 = pt2
    x3, y3 = pt

    x, y, c = 0.0, 0.0, 0.0
    if x2 - x1 != 0.0:
        m = (y2-y1) / (x2-x1)
        x = ((m**2)*x1 - m*y1 + x3 + m*y3) / (m**2 + 1)
        y = m*(x - x1) + y1
        v1 = (x2-x1, y2-y1)
        v2 = (x3-x, y3-y)
        c = np.cross(v1, v2)
    else:
        x = x1
        y = y3
        v1 = (0, y2-y1)
        v2 = (x3-x, y3-y)
        c = np.cross(v1, v2)
    dis = euc_distance((x3, y3), (x, y))
    if c < 0:
        return (-1)*dis
    else:
        return dis


def calc_cte_and_idx(pts, pt):
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

    cte = cross_track_error(pt1, pt2, pt)

    return cte, min_idx


def ref_to_csp(ref_path):
    x_list, y_list = zip(*ref_path)
    csp = cubic_spline_planner.Spline2D(x_list, y_list)
    return csp


def ref_to_max_v(ref_path, precision, v_offset, min_v, ref_v, speed_limit):
    csp = ref_to_csp(ref_path)
    max_v = []

    for i in range(len(ref_path)):
        s = i * precision
        k = csp.calc_curvature(s)
        if k != 0.0 and k is not None:
            R = abs(1.0 / k)
            curvature_v = math.sqrt(127 * 0.25 * R) - v_offset
        else:
            curvature_v = 300

        v = max(min_v, min(curvature_v, min(ref_v, speed_limit)))

        max_v.append(v * KPH_TO_MPS)

    return max_v


def new_velocity_plan(ref_v, last_s, s, tl_s, objects_s):
    ref_v *= KPH_TO_MPS
    d_list = []

    traffic_d = tl_s
    margin = 5
    d_list.append(traffic_d)

    d_list.append(last_s-s)

    for object in objects_s:
        for obj in object:
            d = obj[1] - s
            d_list.append(d)

    min_d = min(d_list)

    if not len(d_list) == 0:
        if min_d > 50:
            target_v = ref_v
        else:
            k = 0.8
            target_v = k*(min_d - margin + 1)
    else:
        target_v = ref_v

    return target_v


def signal_light_toggle(path, ego_idx, precision, t_map, lmap, stage):
    ids = []
    try:
        forward_range = 40  # meter
        ego_pt = path[ego_idx]
        ego_lanelet_id, _ = lanelet_matching(
            t_map.tiles, t_map.tile_size, ego_pt)
        left_lanelet = lmap.lanelets[ego_lanelet_id]['adjacentLeft']
        right_lanelet = lmap.lanelets[ego_lanelet_id]['adjacentRight']

        for distance in range(forward_range+1):
            forward_pt = path[ego_idx+int(distance/precision)]
            id, _ = lanelet_matching(t_map.tiles, t_map.tile_size, forward_pt)
            ids.append(id)

        forward_lanelet_ids = np.array(ids)
        # 0: normal, 1: left, 2: right
        # if forward_lanelet_id == left_lanelet or (stage == 1 and ):
        if np.any(forward_lanelet_ids == left_lanelet):
            change = 1
        elif np.any(forward_lanelet_ids == right_lanelet):
            change = 2
        else:
            change = 0
        # print(change)
        return change
    except IndexError:
        pass

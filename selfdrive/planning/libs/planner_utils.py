import math

import heapq as hq
import numpy as np
import pymap3d as pm
from scipy.spatial import KDTree

import libs.cubic_spline_planner as cubic_spline_planner
from libs.quadratic_spline_interpolate import QuadraticSplineInterpolate
from selfdrive.visualize.rviz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
M_TO_IDX = 2
IDX_TO_M = 0.5

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

def get_my_neighbor(lanelets, my_id):
    l_id = lanelets[my_id]['adjacentLeft']
    if l_id != None and lanelets[l_id]['successor'] is not None:
        l_front_id = lanelets[l_id]['successor'][0] if len(lanelets[l_id]['successor']) > 0 else None
    else:
        l_front_id = None
    #TODO: have to look left_front, right_front
    #lanelets[l_id]['successor']
    r_id = lanelets[my_id]['adjacentRight']
    if r_id != None and lanelets[r_id]['successor'] is not None:
        r_front_id = lanelets[r_id]['successor'][0] if len(lanelets[r_id]['successor']) > 0 else None
    else:
        r_front_id = None
    return ((l_id,l_front_id),(r_id,r_front_id))

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
        obs_idx = obs_len if obs_len < len(
            local_path_from_now) else len(local_path_from_now)-1
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
                crosswalks = data['crosswalk'][0]
                if isinstance(crosswalks[0], list):
                    for arr in crosswalks:
                        crosswalk.extend(arr)
                else:
                    crosswalk.append(crosswalks)

    now_cw_idx = math.inf

    for cw_wp in crosswalk:
        idx = local_point.query(cw_wp, 1)[1]
        if idx < now_cw_idx:
            now_cw_idx = idx
    return now_cw_idx


def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points


def interpolate(points, precision):

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


def ref_interpolate(points, precision):
    points = filter_same_points(points)
    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))
    itp_points = []
    for ds in np.arange(0.0, itp.s[-1], precision):
        x, y = itp.calc_position(ds)
        itp_points.append((float(x), float(y)))

    return itp_points, itp.s[-1]


def id_interpolate(non_intp, intp, non_intp_id):
    itp_ids = []
    non_intp_idx = 0
    for wp in intp:
        calc_non_intp_idx = calc_idx(non_intp, wp)
        if calc_non_intp_idx > non_intp_idx:
            non_intp_idx = calc_non_intp_idx
        n_id = non_intp_id[non_intp_idx]
        itp_ids.append(n_id)
    return itp_ids


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
            final_id_path.append(str(id))
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
    else:
        pt1 = pts[min_idx]

    return min_idx


def ref_to_csp(ref_path):
    x_list, y_list = zip(*ref_path)
    csp = cubic_spline_planner.Spline2D(x_list, y_list)
    return csp


def max_v_by_curvature(forward_curvature, ref_v, min_v, cur_v):
    threshold = 160
    return_v = ref_v

    # Determine the multiplier based on cur_v
    if 0*KPH_TO_MPS <= cur_v < 20*KPH_TO_MPS:
        coeffect = 0.09
    elif 20*KPH_TO_MPS <= cur_v < 30*KPH_TO_MPS:
        coeffect = 0.11
    else:
        coeffect = 0.15

    if forward_curvature < threshold:
        return_v = ref_v - (abs(threshold - forward_curvature) * coeffect)
        return_v = return_v if return_v > min_v else min_v

    return return_v * KPH_TO_MPS

def calculate_v_by_curvature(lane_information, ref_v, min_v, cur_v):
    #lane information -> [1]:forward direction, [3]:curvature
    max_curvature = 600
    min_curvature = 0
    if lane_information[3] < min_curvature:
        lane_information[3] = min_curvature
    elif lane_information[3] > max_curvature:
        lane_information[3] = max_curvature
    
    normalized_curvature = (lane_information[3] - min_curvature) / (max_curvature - min_curvature)

    decel = (ref_v - min_v) * (1 - normalized_curvature)
    return_v = ref_v - decel
    # print("return-v:", return_v, "cur_v:",cur_v)
    if cur_v - return_v*KPH_TO_MPS > 8*0.1:
        return_v = cur_v*MPS_TO_KPH - (8*0.1*MPS_TO_KPH)
        # print("!!!!!!!return-v:", return_v, "cur_v:",cur_v)

    if return_v > ref_v:
        return_v = ref_v
    # print(decel, return_v)
    if lane_information[1]==1:
        return_v = min(return_v, max(return_v, 25))    
    return return_v*KPH_TO_MPS


def get_a_b_for_curv(min, ignore):
    # a = -90 / (min-ignore)
    # b = 60-(ignore*a)
    a = 4.05
    b = -26.25
    return a, b

def get_a_b_for_blinker(min, ignore):
    # a = -40/(min-ignore)
    # b = 60-(ignore*a)
    a = 4.05
    b = 1.25
    return a,b

def get_blinker(idx, lanelets, ids, my_neighbor_id, vEgo):
    a, b = get_a_b_for_blinker(10*KPH_TO_MPS, 50*KPH_TO_MPS)
    lf = int(min(idx+110, max(idx+(a*vEgo+b)*M_TO_IDX, idx+20))) # 10m ~ 55m
    if lf < 0:
        lf = 0
    elif lf > len(ids)-1:
        lf = len(ids)-1
    next_id = ids[lf].split('_')[0]
    if next_id in my_neighbor_id[0]:# or (lanelets[next_id]['laneNo'] == 91 or lanelets[next_id]['laneNo'] == 92):
        return 1, next_id
    elif next_id in my_neighbor_id[1]:
        return 2, next_id
        
    return 0, None


def compare_id(lh_id, my_neighbor_id):
    if lh_id in my_neighbor_id[0] or lh_id in my_neighbor_id[1]:
        return False
    else:
        return True

def get_forward_curvature(idx, path, yawRate, vEgo, blinker, lanelets, now_id):
    # ws = int((1.5*vEgo)+70)
    ws = int(14.4*vEgo+80)
    # a, b = get_a_b_for_curv(10*KPH_TO_MPS, 50*KPH_TO_MPS)
    x = []
    y = []
    trajectory = []
    id_list = None

    #lf = int(min(idx+60, max(idx+(a*vEgo+b)*M_TO_IDX, idx-30))) # -15m~30m
    lf = int(idx-40)
    if lf < 0:
        lf = 0
    elif lf > len(path)-1:
        lf = idx

    if lf+ws < len(path):
        x = [v[0] for v in path[lf:lf+ws]]
        y = [v[1] for v in path[lf:lf+ws]]
        trajectory = path[lf:lf+ws]

    else:
        x = [v[0] for v in path[lf:]]
        y = [v[1] for v in path[lf:]]
        trajectory = path[lf:]

    x = np.array([(v-x[0]) for v in x])
    y = np.array([(v-y[0]) for v in y])

    # For Trajectory plotting
    origin_plot = np.vstack((x, y))
    rotation_radians = math.radians(-yawRate) + math.pi/2
    rotation_mat = np.array([[math.cos(rotation_radians), -math.sin(rotation_radians)],
                             [math.sin(rotation_radians), math.cos(rotation_radians)]])
    rot_x, rot_y = list(rotation_mat@origin_plot)
    
    # Calculate curvature by trajectory
    if len(x) > 2:
        cr = np.polyfit(rot_y, rot_x, 2)
        if cr[0] != 0:
            curvature = ((1+(2*cr[0]+cr[1])**2) ** 1.5)/np.absolute(2*cr[0])
        else:
            curvature = 1000
    else:
        curvature = 1000

    if blinker > 0:
        curvature = 1000

    if lanelets[now_id]['uTurn'] == True:
        curvature = -40
    # if lanelets[now_id]['intersection'] == True:
    #     curvature = curvature * 1.5
    # print("curvature : ", curvature)
    return curvature, rot_x, rot_y, trajectory

def get_lane_change_point(ids, idx, my_neighbor_id):
    for i, id in enumerate(ids[idx:]):
        if id.split('_')[0] in my_neighbor_id[0] or id.split('_')[0] in my_neighbor_id[1]:
            return idx+i
    return 999999
     
def get_renew_path(ids, change_direction, lane_change_idx, lanelets, local_path_from_now, local_path_before_now):
    target_lane_id = None
    renew_path = None
    renew_id = None
    #check renew possibility
    if change_direction == 1 : #Left
        target_lane_id = lanelets[ids[lane_change_idx].split('_')[0]]['adjacentRight']
        if target_lane_id == None:
            return None,None
    elif change_direction == 2 :
        target_lane_id = lanelets[ids[lane_change_idx].split('_')[0]]['adjacentLeft']
        if target_lane_id == None:
            return None,None
    else:
        return None,None
    
    before_ids = []
    renew_ids = []
    if target_lane_id != None:
        #local_path_from_now = local_path_from_now[:idx+2]
        before_lane_id = ids[lane_change_idx-15].split('_')[0]
        before_path = exchange_waypoint(lanelets[before_lane_id]['waypoints'], local_path_before_now)
        for _ in range(len(before_path)):
            before_ids.append(before_lane_id)
        renew_path = exchange_waypoint(lanelets[target_lane_id]['waypoints'], local_path_from_now)
        for _ in range(len(renew_path)):
            renew_ids.append(target_lane_id)

    else:
        return None,None
    
    return before_path+renew_path, before_ids+renew_ids

def get_lane_change_path(ids, change_direction, l_idx, lanelets, local_path_point2point):
    target_lane_id = None
    renew_path = None
    # renew_id = None
    #get 50m front target lane id 
    if change_direction == 1 : #Left
        target_lane_id = lanelets[ids[l_idx+100].split('_')[0]]['adjacentLeft']
        if target_lane_id == None:
            return None,None
    elif change_direction == 2 :
        target_lane_id = lanelets[ids[l_idx+100].split('_')[0]]['adjacentRight']
        if target_lane_id == None:
            return None,None
    else:
        return None,None
    
    # before_ids = []
    renew_ids = []
    if target_lane_id != None:
        #local_path_point2point = local_path_point2point[:idx+2]
        # before_lane_id = ids[l_idx+100-15].split('_')[0]
        # before_path = exchange_waypoint(lanelets[before_lane_id]['waypoints'], local_path_before_now)
        # for _ in range(len(before_path)):
        #     before_ids.append(before_lane_id)
        renew_path = exchange_waypoint(lanelets[target_lane_id]['waypoints'], local_path_point2point)
        for _ in range(len(renew_path)):
            renew_ids.append(target_lane_id)
    else:
        return None,None
    
    return renew_path, renew_ids

def get_forward_direction(lanelets, next_id):  # (global_path, i, ws=200):
    # return direction - 0:straight, 1:left, 2:right,3:left lane change, 4:right lane change, 5:U-turn
    link = lanelets[next_id]
    p = (link['waypoints'][len(link['waypoints'])//5][0],
         link['waypoints'][len(link['waypoints'])//5][1])
    q = (link['waypoints'][len(link['waypoints'])//2][0],
         link['waypoints'][len(link['waypoints'])//2][1])
    r = (link['waypoints'][len(link['waypoints'])//5*4][0],
         link['waypoints'][len(link['waypoints'])//5*4][1])

    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])

    if link['uTurn']:
        return 'U'
    if link['rightTurn']:
        return 'R'
    if link['leftTurn']:
        return 'L'
    threshold = 10
    if -threshold < val < threshold:
        return 'S'  # collinear
    elif val <= -threshold:
        return 'L'  # left turn
    elif val >= threshold:
        return 'R'  # right turn


def set_lane_ids(lst):
    lst = [elem.split("_")[0] for elem in lst]

    lane_ids = [lst[0]]

    for i in range(1, len(lst)):
        if lst[i] != lst[i-1]:
            lane_ids.append(lst[i])

    return lane_ids

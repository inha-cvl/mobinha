import math
import time

import heapq as hq
import numpy as np
import pymap3d as pm
from scipy.spatial import KDTree

import libs.cubic_spline_planner as cubic_spline_planner
from libs.quadratic_spline_interpolate import QuadraticSplineInterpolate
from selfdrive.visualize.rviz_utils import *

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
HZ = 10

blinker_start_time = None
blinker_minimum_duration = 2  # 최소 블링커 지속 시간 (초)
blinker_status = 0  # 현재 블링커 상태 (0: 꺼짐, 1: 좌, 2: 우)
blinker_target_id = None

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
    
    def get_front_id(current_id):
        """Helper function to get the front id for a given lanelet id."""
        if current_id is not None and lanelets[current_id]['successor'] is not None:
            if set(lanelets[current_id]['successor']) & set(lanelets[my_id]['successor']):
                return None
            else:
                if len(lanelets[current_id]['successor']) == 1:
                    return lanelets[current_id]['successor'][0]
                elif len(lanelets[current_id]['successor']) > 1:
                    for id in lanelets[current_id]['successor']:
                        if lanelets[id]['laneNo'] == lanelets[current_id]['laneNo']:
                            return id
        return None

    # Initialize lanelet ids
    l_id, l_front_id, r_id, r_front_id = None, None, None, None
    
    l_id = lanelets[my_id]['adjacentLeft']
    l_front_id = get_front_id(l_id)
    
    r_id = lanelets[my_id]['adjacentRight']
    r_front_id = get_front_id(r_id)

    return ((l_id, l_front_id), (r_id, r_front_id))

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


def get_nearest_stopline(lanelets, stoplines, nowID, head_lane_ids, local_point):
    stopline = []
    sl_id = None
    if len(lanelets[nowID]['stoplineID']) > 0:
        sl_id = lanelets[nowID]['stoplineID']
    else:
        for lanelet_id in head_lane_ids:
            if len(lanelets[lanelet_id]['stoplineID']) > 0:
                sl_id = lanelets[lanelet_id]['stoplineID']
                break
    if sl_id is not None:
        stopline = stoplines[sl_id[0]]

    now_sl_idx = math.inf

    for sl_wp in stopline:
        idx = local_point.query(sl_wp, 1)[1]
        if idx < now_sl_idx:
            now_sl_idx = idx
    return now_sl_idx, stopline


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

    return itp_points, itp.s[-1], yaw, k

from scipy.interpolate import UnivariateSpline

def ref_interpolate_2d(points, precision, smoothing=0):
    points = filter_same_points(points)
    wx, wy = zip(*points)

    # Create a cumulative distance array
    dist = [0.0]
    for i in range(1, len(wx)):
        dist.append(dist[-1] + np.sqrt((wx[i] - wx[i-1])**2 + (wy[i] - wy[i-1])**2))
    total_distance = dist[-1]

    # Use 2nd order (quadratic) UnivariateSpline for interpolation
    sx = UnivariateSpline(dist, wx, k=2, s=smoothing)
    sy = UnivariateSpline(dist, wy, k=2, s=smoothing)

    # Generate interpolated points
    itp_points = []
    for d in np.arange(0, total_distance, precision):
        itp_points.append((float(sx(d)), float(sy(d))))

    return itp_points, total_distance

from scipy.ndimage import gaussian_filter1d

def gaussian_smoothing_2d(points, sigma=1):
    wx, wy = zip(*points)
    smoothed_wx = gaussian_filter1d(wx, sigma=sigma)
    smoothed_wy = gaussian_filter1d(wy, sigma=sigma)
    
    return list(zip(smoothed_wx, smoothed_wy))

def smooth_compute_yaw_and_curvature(points, precision):
    # Apply Gaussian smoothing
    smoothed_path = gaussian_smoothing_2d(points)
    
    # Extract x and y from smoothed path
    wx, wy = zip(*smoothed_path)
    
    # Create an interpolator object
    itp = QuadraticSplineInterpolate(list(wx), list(wy))
    
    yaw = []
    k = []
    
    # Compute yaw and curvature for each point in the smoothed path
    for ds in np.arange(0.0, itp.s[-1], precision):
        yaw.append(itp.calc_yaw(ds))
        k.append(itp.calc_curvature(ds))
        
    return smoothed_path, yaw, k

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
    tmp_non_intp_idx = 0
    for wp in intp:
        calc_non_intp_idx = calc_idx(non_intp, wp)

        if abs(calc_non_intp_idx - tmp_non_intp_idx) < 50:
            non_intp_idx = calc_non_intp_idx
            tmp_non_intp_idx = non_intp_idx
            
        # if calc_non_intp_idx > non_intp_idx:
            # non_intp_idx = calc_non_intp_idx

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

def get_forward_direction(lanelets, now_id, head_lane_ids):  # (global_path, i, ws=200):
    # return direction - 0:straight, 1:left, 2:right,3:left lane change, 4:right lane change, 5:U-turn
    lane_ids = [now_id]+head_lane_ids
    for id_ in lane_ids:
        
        if not lanelets[id_]['leftTurn'] and not lanelets[id_]['rightTurn'] and len(lanelets[id_]['crosswalkID']) > 1:
            return 'S'

        if lanelets[id_]['intersection']:
            if lanelets[id_]['leftTurn']:

                return 'L'
            elif lanelets[id_]['rightTurn']:

                return 'R'
            else:

                return 'S'
            
        if lanelets[id_]['rightTurn']:
            return 'R'
        
    return 'S'
 
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

def calculate_v_by_curvature(lane_information, ref_v, min_v, cur_v): # info, kph, kph, mps
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
    if lane_information[3] < max_curvature:
        if cur_v - return_v*KPH_TO_MPS > 5/HZ: # smooth deceleration
            return_v = cur_v*MPS_TO_KPH - (5/HZ*MPS_TO_KPH)
            # print("decel return-v:", return_v, "cur_v:",cur_v*MPS_TO_KPH)
        elif cur_v*MPS_TO_KPH > min_v and return_v*KPH_TO_MPS - cur_v > 5/HZ: # smooth acceleration
            return_v = cur_v*MPS_TO_KPH + (5/HZ*MPS_TO_KPH)
            # print("accel return-v:", return_v, "cur_v:",cur_v*MPS_TO_KPH)
    if return_v > ref_v:
        return_v = ref_v

    # if lane_information[1]==1:
    #     return_v = min(return_v, max(return_v, 25))    
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
    a = 3.957
    b = 10
    return a,b


def get_blinker(idx, lanelets, ids, my_neighbor_id, vEgo, M_TO_IDX, splited_local_id):#, local_id, change_target_id, change_lane_flag): 
    #TODO: CHECK FLAG    
    global blinker_start_time, blinker_status, blinker_target_id
 
    a, b = get_a_b_for_blinker(10*KPH_TO_MPS, 50*KPH_TO_MPS)
    lf = int(min(idx+110, max(idx+(a*vEgo+b)*M_TO_IDX, idx+20))) # 15m ~ 65m
    ld = int(min(idx+130, max(idx+(a*vEgo+b)*M_TO_IDX, idx+40))) # lookahead distance, lf보다 조금 더 먼 거리를 보게 함 
    # left_turn_lane = [1, 2, 91]

    if lf < 0:
        lf = 0
    elif lf > len(ids)-1:
        lf = len(ids)-1
    next_id_1 = ids[lf].split('_')[0]

    if ld < 0:
        ld = 0
    elif ld > len(ids)-1:
        ld = len(ids)-1
    next_id_2 = ids[ld].split('_')[0]

    # 차선 변경할 때
    if next_id_1 in my_neighbor_id[0]:
        # or (lanelets[next_id]['laneNo'] == 91 or lanelets[next_id]['laneNo'] == 92):
        print("now Lchange")
        return 1, next_id_1
    elif next_id_1 in my_neighbor_id[1]:
        print("now Rchange")
        return 2, next_id_1
    
    # 다음링크가 회전차로일 때
    elif max(lanelets[next_id_2]['yaw']) - min(lanelets[next_id_2]['yaw']) > 0.5 and lanelets[next_id_2]['intersection'] == True:
        if lanelets[next_id_2]['rightTurn'] == True:
            print(next_id_2)
            print("next Rturn")
            return 2, next_id_2
        else:
            print(next_id_2)
            print("next Lturn")
            return 1, next_id_2
        
    # 다음링크가 포켓차로일 때
    elif max(lanelets[next_id_1]['yaw']) - min(lanelets[next_id_1]['yaw']) > 0.25 and lanelets[splited_local_id]['direction'] != None:
        if lanelets[next_id_1]['direction'] == 'L':
            print(next_id_1)
            print("next Lpocket")
            return 1, next_id_1
        else:  # lanelets[next_id_1]['direction'] == 'R'
            print(next_id_1)
            print("next Rpocket")
            return 2, next_id_2

    # 다음링크가 yaw값이 많이 변할 때
    # elif max(lanelets[next_id_2]['yaw']) - min(lanelets[next_id_2]['yaw']) > 0.1:
    #     # 다음링크가 교차로를 통과하거나 좌회전 가능차로일 때
    #     if lanelets[next_id_2]['laneNo'] != lanelets[splited_local_id]['laneNo'] or lanelets[next_id_2]['leftTurn'] == True:
    #         print(next_id_2)
    #         return 1, next_id_2
    
    # 현재링크가 회전차로일 때
    elif max(lanelets[splited_local_id]['yaw']) - min(lanelets[splited_local_id]['yaw']) > 0.5 and lanelets[splited_local_id]['intersection'] == True:
        # 현재링크가 우회전일 때
        if lanelets[splited_local_id]['rightTurn'] == True: # and splited_local_id != '5':
            print(splited_local_id)
            print("now Rturn")
            return 2, next_id_2
        # 현재링크가 좌회전일 때
        else:
            print(splited_local_id)
            print("now Lturn")
            return 1, next_id_2
    
     # 현재링크가 포켓차로일 때
    elif max(lanelets[splited_local_id]['yaw']) - min(lanelets[splited_local_id]['yaw']) > 0.25 and lanelets[splited_local_id]['direction'] != None:
        # 현재링크가 좌포켓일 때
        if lanelets[splited_local_id]['direction'] == 'L': # and splited_local_id != '5':
            print(splited_local_id)
            print("now Lpocket")
            return 1, next_id_1
        # 현재링크가 우포켓일 때
        elif lanelets[splited_local_id]['direction'] == 'R' and lanelets[splited_local_id]['laneNo'] >= lanelets[next_id_1]['laneNo']:
            print(splited_local_id)
            print("now Rpocket")
            return 2, next_id_1

    # if change_lane_flag:  # if flag is True, keep the blinker on
    #     if change_target_id in my_neighbor_id[0]:
    #         return 1 , change_target_id
    #     elif change_target_id in my_neighbor_id[1]:
    #         return 2, change_target_id

    return 0, None


def compare_id(lh_id, my_neighbor_id):
    if lh_id in my_neighbor_id[0] or lh_id in my_neighbor_id[1]:
        return False
    else:
        return True

def get_forward_curvature(idx, path, yawRate, vEgo, blinker, lanelets, now_id, next_id, M_TO_IDX):
    # ws = int((1.5*vEgo)+70)
    # ws = int(14.4*vEgo+80)
    ws = int(1.014*vEgo**2 - 0.776*vEgo + 95)
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
        curvature = 0
    # if (lanelets[next_id]['laneNo'] == 91 or lanelets[next_id]['laneNo'] == 92):
    #     curvature = 500
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
    lanechange_point_idx = (30+30)*2
    if change_direction == 1 : #Left
        target_lane_id = lanelets[ids[l_idx+lanechange_point_idx].split('_')[0]]['adjacentLeft']
        if target_lane_id == None:
            return None,None
    elif change_direction == 2 :
        target_lane_id = lanelets[ids[l_idx+lanechange_point_idx].split('_')[0]]['adjacentRight']
        if target_lane_id == None:
            return None,None
    else:
        return None,None
    
    # before_ids = []
    renew_ids = []
    if target_lane_id != None:
        #local_path_point2point = local_path_point2point[:idx+2]
        # before_lane_id = ids[l_idx+130-15].split('_')[0]
        # before_path = exchange_waypoint(lanelets[before_lane_id]['waypoints'], local_path_before_now)
        # for _ in range(len(before_path)):
        #     before_ids.append(before_lane_id)
        renew_path = exchange_waypoint(lanelets[target_lane_id]['waypoints'], local_path_point2point)
        for _ in range(len(renew_path)):
            renew_ids.append(target_lane_id)
    else:
        return None,None
    
    return renew_path, renew_ids

def set_lane_ids(lst):
    lst = [elem.split("_")[0] for elem in lst]

    lane_ids = [lst[0]]

    for i in range(1, len(lst)):
        if lst[i] != lst[i-1]:
            lane_ids.append(lst[i])

    return lane_ids

def findMyLinkIdx(lanelets, l_id, ego_x, ego_y):
    my_link_wps = KDTree(lanelets[l_id]['waypoints'])
    link_idx = my_link_wps.query((ego_x, ego_y), 1)[1]
    return link_idx

def removeVegetationFromRoadside(lanelets, l_id, link_idx):
    lane_no = lanelets[l_id]['laneNo']
    length = lanelets[l_id]['length']
    my_distance = lanelets[l_id]['s'][link_idx]
    left_id = lanelets[l_id]['adjacentLeft']
    succ_id = lanelets[l_id]['successor'][0] if len(lanelets[l_id]['successor']) > 0 else None
    pre_id = lanelets[l_id]['predecessor'][0] if len(lanelets[l_id]['predecessor']) > 0 else None
    succ_pre_cnt = len(lanelets[succ_id]['predecessor']) if succ_id else 1
    pre_left_id = lanelets[pre_id]['adjacentLeft'] if pre_id else None

    if lane_no == 1 and left_id and (lanelets[left_id]['laneNo'] == 91 or lanelets[left_id]['laneNo'] == 92):
        if succ_pre_cnt >= 2 and my_distance > length - 40:
            lane_position = 1
        elif pre_left_id and lanelets[pre_left_id]['laneNo'] == 91:
            lane_position = 2
        elif my_distance < 20:
            lane_position = 1
        else:
            lane_position = 2 # [2] | |@| |
    elif (lane_no == 1 and lanelets[l_id]['adjacentLeft'] == None) or lane_no == 91:
        lane_position = 1
    elif (lane_no == 3 and lanelets[l_id]['adjacentRight'] == None) or (lane_no == 4 and lanelets[l_id]['adjacentRight'] == None) \
        or (lane_no == 5 and lanelets[l_id]['adjacentRight'] == None) or (lane_no == 6 and lanelets[l_id]['adjacentRight'] == None):
        lane_position = 3
    else:
        lane_position = 2
    
    return lane_position


def extract_path_info(local_path, local_id, lanelets):
    yaw_list = []
    radius_list = []
    k_list = []
    
    for idx, id_str in enumerate(local_id):
        # Extract lanelet_id and waypoint_idx from local_id
        lanelet_id, waypoint_idx = map(int, id_str.split('_'))
        
        # Access corresponding yaw, radius, and k values using lanelet_id and waypoint_idx
        yaw = lanelets[str(lanelet_id)]['yaw'][waypoint_idx]
        k = lanelets[str(lanelet_id)]['k'][waypoint_idx]
        radius = 1 / k if k != 0 else float('inf')
        
        # Append these values to new lists
        yaw_list.append(yaw)
        radius_list.append(radius)
        k_list.append(k)
        
    return yaw_list, radius_list, k_list

def calculate_cte(pointA, pointB, pointP):
    Ax, Ay = pointA
    Bx, By = pointB
    Px, Py = pointP

    numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
    denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
    # return numerator / denominator if denominator != 0 else 0
    cte = numerator / denominator if denominator != 0 else 0
    # Calculate cross product to find the sign
    cross_product = (Bx - Ax) * (Py - Ay) - (By - Ay) * (Px - Ax)
    
    if cross_product > 0:
        return -cte  # Point P is on the left side of line AB
    elif cross_product < 0:
        return cte  # Point P is on the right side of line AB
    else:
        return 0  # Point P is on the line AB

def estimate_theta(path, index):
    point_current = path[index]
    point_next = path[index + 1] if index + 1 < len(path) else path[index]
    
    dx = point_next[0] - point_current[0]
    dy = point_next[1] - point_current[1]
    
    theta = np.arctan2(dy, dx)
    
    return theta

def is_car_inside_combined_road(obstacle_position, lanelet, prevID, nowID, nextID):
                                # prevLeftBound, prevRightBound, 
                                # nowLeftBound, nowRightBound, nextLeftBound, nextRightBound):
    def find_edge_id(flag, adjacent_id, lanelets):
        if flag == 'l':
            while adjacent_id is not None:
                if lanelets[adjacent_id]['adjacentLeft'] is None:
                    break
                adjacent_id = lanelets[adjacent_id]['adjacentLeft']
        elif flag == 'r':
            while adjacent_id is not None:
                if lanelets[adjacent_id]['adjacentRight'] is None:
                    break
                adjacent_id = lanelets[adjacent_id]['adjacentRight']
        return adjacent_id
    
    def get_direction(chunk):
        start = chunk[0]
        end = chunk[-1]
        return (end[0] - start[0], end[1] - start[1])
    def sort_key(chunk, all_chunks):
        directions = [get_direction(c) for c in all_chunks]
        avg_direction = (sum(d[0] for d in directions) / len(directions), sum(d[1] for d in directions) / len(directions))
        start = chunk[0]
        return start[0] * avg_direction[0] + start[1] * avg_direction[1]
    def get_ordered_chunks(chunks):
        sorted_chunks = sorted(chunks, key=lambda chunk: sort_key(chunk, chunks))
        return sorted_chunks
        
    def create_and_flatten_polygon(leftBound, rightBound):
        if leftBound is None or rightBound is None:
            return []
        
        ordered_leftBound = get_ordered_chunks(leftBound)
        ordered_rightBound = get_ordered_chunks(rightBound)

        # Flatten the coordinates inside the chunks
        flat_ordered_left = [coord for sublist in ordered_leftBound for coord in sublist]
        # Flatten the coordinates inside the chunks and then reverse their order
        flat_ordered_right = [coord for sublist in ordered_rightBound for coord in sublist][::-1]
        
        # Create the polygon using the flattened coordinates
        polygon = flat_ordered_left + flat_ordered_right
        return polygon
    
    def is_point_inside_polygon(pt, poly):
        x, y = pt
        oddNodes = False
        j = len(poly) - 1  # The last vertex is the 'previous' one to start with

        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if yi < y and yj >= y or yj < y and yi >= y:
                if xi + (y - yi) / (yj - yi) * (xj - xi) < x:
                    oddNodes = not oddNodes
            j = i
        return oddNodes

    # Finding the most left and right boundaries for each lanelet ID
    prevLeftBound = lanelet[find_edge_id('l', prevID, lanelet)]['leftBound'] if prevID else None
    prevRightBound = lanelet[find_edge_id('r', prevID, lanelet)]['rightBound'] if prevID else None
    nowLeftBound = lanelet[find_edge_id('l', nowID, lanelet)]['leftBound'] if nowID else None
    nowRightBound = lanelet[find_edge_id('r', nowID, lanelet)]['rightBound'] if nowID else None
    nextLeftBound = lanelet[find_edge_id('l', nextID, lanelet)]['leftBound'] if nextID else None
    nextRightBound = lanelet[find_edge_id('r', nextID, lanelet)]['rightBound'] if nextID else None

    # Create and flatten polygons for each road
    prev_polygon_flat = create_and_flatten_polygon(prevLeftBound, prevRightBound)
    now_polygon_flat = create_and_flatten_polygon(nowLeftBound, nowRightBound)
    next_polygon_flat = create_and_flatten_polygon(nextLeftBound, nextRightBound)

    # Determine if the obstacle is inside any of the flattened road polygons
    road1_result = is_point_inside_polygon(obstacle_position, prev_polygon_flat) if prev_polygon_flat else False
    road2_result = is_point_inside_polygon(obstacle_position, now_polygon_flat) if now_polygon_flat else False
    road3_result = is_point_inside_polygon(obstacle_position, next_polygon_flat) if next_polygon_flat else False

    return prev_polygon_flat, now_polygon_flat, next_polygon_flat, road1_result or road2_result or road3_result # if just one true is true return true

def get_crosswalk_points(lanelets, surfacemarks, nowID, head_lane_ids):
    polygon_points = []
    if len(lanelets[nowID]['crosswalkID']) > 0:
        lanelet_id = nowID
    else:
        for lanelet_id in head_lane_ids:
            if len(lanelets[lanelet_id]['crosswalkID']) > 0:
                break
    crosswalk_ids = lanelets[lanelet_id]['crosswalkID']
    for s_id in lanelets[lanelet_id]['crosswalkID']:
        polygon_points.extend(surfacemarks[s_id])
    return crosswalk_ids, polygon_points
# import rospy
# from visualization_msgs.msg import Marker

# def create_road_marker(polygon_coords, marker_id, color, frame_id="world"):
#     marker = Marker()
#     marker.header.frame_id = frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.id = marker_id
#     marker.type = Marker.LINE_STRIP
#     marker.action = Marker.ADD
    
#     # Define the marker scale, color, etc.
#     marker.scale.x = 0.5  # Width of the line
#     marker.color.a = 1.0  # Opacity
#     marker.color.r = color[0]
#     marker.color.g = color[1]
#     marker.color.b = color[2]
    
#     for coord in polygon_coords:
#         marker.points.append(Point(x=coord[0], y=coord[1], z=0))
#         # point = marker.points.append()
#         # point.x = coord[0]
#         # point.y = coord[1]
#         # point.z = 0  # Assuming the roads are flat
    
#     return marker

def is_obstacle_inside_polygon(surfacemarks, crosswalk_ids, obstacle_list):
    def is_point_inside_polygon(point, polygon):
        x, y = point
        inside = False

        for i in range(len(polygon)):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % len(polygon)]
            if y > min(y1, y2) and y <= max(y1, y2) and x <= max(x1, x2):
                if y1 != y2:
                    xinters = (y - y1) * (x2 - x1) / (y2 - y1) + x1
                if x1 == x2 or x <= xinters:
                    inside = not inside

        return inside
    for obs in obstacle_list:
        point = (obs[3], obs[4])  # Assuming obs[3] is x and obs[4] is y
        for s_id in crosswalk_ids:
            polygon_points = surfacemarks[s_id]
            if is_point_inside_polygon(point, polygon_points):
                return True  # Return True if any obstacle is inside any polygon

    return False  # Return False if no obstacle is inside any polygon
            
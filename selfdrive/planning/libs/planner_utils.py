import math

import heapq as hq
import numpy as np
from scipy.spatial import KDTree

from libs.quadratic_spline_interpolate_scipy import QuadraticSplineInterpolateFast
from selfdrive.visualize.rviz_utils import *

import shapely.geometry as sh
from itertools import groupby
from scipy.interpolate import UnivariateSpline
from scipy.ndimage import gaussian_filter1d

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6
HZ = 10


def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)

def lanelet_matching(tiles, tile_size, t_pt): # fast enough
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tiles.get((row+i, col+j))
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
    # print("Current lane id", nowID)
    # print("Current stopline", sl_id)
    if sl_id is not None:
        stopline = stoplines[sl_id[0]]

    now_sl_idx = math.inf

    for sl_wp in stopline:
        idx = local_point.query(sl_wp, 1)[1]
        if idx < now_sl_idx:
            now_sl_idx = idx

    # print("stopline_idx: ", now_sl_idx)
    # print("stopline_wps: ", stopline)
    return now_sl_idx, stopline

def filter_same_points(points):
    return [next(g) for _, g in groupby(points)]

def ref_interpolate_2d(points, precision, smoothing=0.0):
    pts = np.asarray(filter_same_points(points), dtype=float)   # (N,2)

    seg = np.hypot(np.diff(pts[:,0]), np.diff(pts[:,1]))  # (N-1,)
    dist = np.hstack(([0.0], np.cumsum(seg)))             # (N,)
    total_distance = float(dist[-1])

    sx = UnivariateSpline(dist, pts[:,0], k=2, s=smoothing)
    sy = UnivariateSpline(dist, pts[:,1], k=2, s=smoothing)

    d_vals = np.arange(0.0, total_distance, precision)    # (M,)

    xi = sx(d_vals)
    yi = sy(d_vals)

    itp_points = list(zip(xi.astype(float), yi.astype(float)))
    return itp_points, total_distance

def gaussian_smoothing_2d(points, sigma=1.0):
    arr = np.asarray(points, dtype=float)  
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("points must be [(x,y), ...]")

    smoothed_x = gaussian_filter1d(arr[:, 0], sigma=sigma, mode="nearest")
    smoothed_y = gaussian_filter1d(arr[:, 1], sigma=sigma, mode="nearest")

    return np.column_stack((smoothed_x, smoothed_y))

def smooth_compute_yaw_and_curvature(points, precision, sigma=1.0):
    smoothed_path = gaussian_smoothing_2d(points, sigma)
    
    xs, ys = zip(*smoothed_path)          
    itp = QuadraticSplineInterpolateFast(xs, ys)

    s_query = np.arange(0.0, itp.s_end, precision)
    yaw, k  = itp.yaw_and_curvature(s_query)

    
    return smoothed_path, yaw, k

def id_interpolate(non_intp, intp, non_intp_id, enforce_forward=True):
    kdt = KDTree(non_intp)                   

    _, idxs = kdt.query(intp)                     

    if enforce_forward:
        idxs = np.maximum.accumulate(idxs)

    return [non_intp_id[i] for i in idxs]

def node_matching(lanelet, l_id, l_idx): # fast enough
    node_id = l_id

    if lanelet[l_id].get('cut_idx') is not None:
        for n, (s_idx, e_idx) in enumerate(lanelet[l_id]['cut_idx']):
            if l_idx >= s_idx and l_idx < e_idx:
                node_id += '_%s' % (n)
                break

    return node_id

def dijkstra(graph, start, finish): # fast enough
    distances = {start: 0}
    previous  = {}
    visited   = set()
    heap      = [(0, start)]

    while heap:
        current_distance, current_node = hq.heappop(heap)
        if current_node in visited:
            continue
        visited.add(current_node)

        if current_node == finish:
            if current_node == start:
                return None

            path = []
            while current_node in previous:
                path.append(current_node)
                current_node = previous[current_node]
            path.append(start)
            path.reverse()
            return (path, distances[finish])

        for neighbor, weight in graph.get(current_node, {}).items():
            if neighbor in visited or neighbor == start:
                continue
            distance = current_distance + weight
            if neighbor not in distances or distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor]  = current_node
                hq.heappush(heap, (distance, neighbor))

    return None

def node_to_waypoints(lanelet, shortest_path):
    final_path = []
    final_id_path = []

    for id in shortest_path:
        alpha_path = []
        split_id = id.split('_')
        if len(split_id) == 2:
            # lanelet[]['cut_idx']: 마이크로-노드(micro-node) 하나가 담당하는 원본 중심선 샘플 범위
            # ex) [[0, 27], [27, 57], [57, 87], [87, 117], [117, 147], ..]
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

def calc_idx(pts, pt): # todo
    # min_dist = float('inf')
    # min_idx = 0

    # for idx, pt1 in enumerate(pts):
    #     dist = euc_distance(pt1, pt)
    #     if dist < min_dist:
    #         min_dist = dist
    #         min_idx = idx

    # if min_idx == len(pts) - 1:
    #     pt1 = pts[min_idx-1]
    # else:
    #     pt1 = pts[min_idx]

    # return min_idx

    pts = np.asarray(pts, dtype=np.float64)   # (N, 2)
    pt  = np.asarray(pt,  dtype=np.float64)   # (2,)

    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("path는 (N, 2) 형상의 점열이어야 합니다.")
    if pt.shape != (2,):
        raise ValueError("pos는 (x, y) 2-차원 좌표여야 합니다.")

    dists_sq = np.sum((pts - pt) ** 2, axis=1)  # (N,)
    return int(np.argmin(dists_sq))


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

def get_blinker_and_targetid(idx, lanelets, ids, my_neighbor_id, vEgo, M_TO_IDX, splited_local_id):
    a, b = get_a_b_for_blinker(10*KPH_TO_MPS, 50*KPH_TO_MPS)
    lf = int(min(idx+110, max(idx+(a*vEgo+b)*M_TO_IDX, idx+20))) # 15m ~ 65m
    ld = int(min(idx+130, max(idx+(a*vEgo+b)*M_TO_IDX, idx+40))) # lookahead distance, lf보다 조금 더 먼 거리를 보게 함 


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

    curr_curv = abs(max(lanelets[splited_local_id]['yaw']) - min(lanelets[splited_local_id]['yaw']))
    next1_curv = abs(max(lanelets[next_id_1]['yaw']) - min(lanelets[next_id_1]['yaw']))
    next2_curv = abs(max(lanelets[next_id_2]['yaw']) - min(lanelets[next_id_2]['yaw']))

    songdo_special_case = ['5']
    
    # 차선 변경
    if next_id_1 in my_neighbor_id[0]:
        print("now Lchange")
        return 1, next_id_1
    elif next_id_1 in my_neighbor_id[1]:
        print("now Rchange")
        return 2, next_id_1
    
    # 다음링크가 회전차로일 때
    elif next2_curv > 0.5 and lanelets[next_id_2]['intersection'] == True and len(lanelets[next_id_2]['successor']) < 2:
        # 우회전
        if lanelets[next_id_2]['rightTurn'] == True and lanelets[next_id_2]['leftTurn'] == False:
            print(next_id_2)
            print("next Rturn")
            return 2, next_id_2
        # 좌회전
        else:
            print(next_id_2)
            print("next Lturn")
            return 1, next_id_2
    
    # 다음링크가 로터리 진입일 때
    elif next1_curv > 0.25 and len(lanelets[next_id_1]['successor']) > 1 and lanelets[next_id_1]['intersection'] == True and lanelets[splited_local_id]['intersection'] == False:
        print(f"{splited_local_id} -> {next_id_1}")
        print("next rotary in")
        return 1, next_id_1
        
    # 다음링크가 포켓차로일 때
    elif next1_curv > 0.25 and len(lanelets[next_id_1]['direction']) > 0:
        # 좌포켓
        if lanelets[next_id_1]['direction'] == 'L':
            print("next Lpocket")
            return 1, next_id_1

        # 우포켓
        elif lanelets[next_id_1]['direction'] == 'R':
            print("next Rpocket")
            return 2, next_id_1
    
    # 곡률 작은 우포켓
    elif next1_curv > 0.07 and lanelets[next_id_1]['laneNo'] > lanelets[splited_local_id]['laneNo']:
        print(next_id_1)
        print("next Rpocket")
        return 2, next_id_1
    
    # 현재링크가 로터리, 다음링크가 로터리 진출차로일 때
    elif curr_curv > 0.25 and len(lanelets[splited_local_id]['successor']) > 1 and lanelets[splited_local_id]['intersection'] == True and curr_curv > next1_curv: 
        print("next rotary out")
        return 2, next_id_1
    
    # 현재링크가 회전차로일 때
    elif curr_curv > 0.5 and lanelets[splited_local_id]['intersection'] == True and len(lanelets[splited_local_id]['successor']) < 2 :
        # 현재링크가 우회전일 때
        if lanelets[splited_local_id]['rightTurn'] == True and lanelets[splited_local_id]['leftTurn'] == False and splited_local_id not in songdo_special_case:
            print(splited_local_id)
            print("now Rturn")
            return 2, next_id_2
        # 현재링크가 좌회전일 때
        else:
            print(splited_local_id)
            print("now Lturn")
            return 1, next_id_2
    
     # 현재링크가 포켓차로일 때
    elif curr_curv > 0.25 and len(lanelets[splited_local_id]['direction']) > 0:
        # 좌포켓
        if lanelets[splited_local_id]['direction'] == 'L' and splited_local_id not in songdo_special_case:
            print("now Lpocket")
            return 1, next_id_1
        # 우포켓
        elif lanelets[splited_local_id]['direction'] == 'R' and lanelets[splited_local_id]['laneNo'] >= lanelets[next_id_1]['laneNo']:
            print("now Rpocket")
            return 2, next_id_1

    return 0, None

def get_stopline_pos(lanelets, splited_local_id):
    if len(lanelets[splited_local_id]['stoplineID']) != 0:
        stopline_pos = lanelets[splited_local_id]['waypoints'][-1]  # [x, y]
    else:
        stopline_pos = None
    return stopline_pos

def compare_id(lh_id, my_neighbor_id):
    if lh_id in my_neighbor_id[0] or lh_id in my_neighbor_id[1]:
        return False
    else:
        return True

def get_forward_curvature(idx, path, yawRate, vEgo, blinker, lanelets, now_id):
    ws = int(1.014*vEgo**2 - 0.776*vEgo + 95)
    x = []
    y = []
    trajectory = []

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

    return curvature, rot_x, rot_y, trajectory

def get_lane_change_point(ids, idx, my_neighbor_id):
    for i, id in enumerate(ids[idx:]):
        if id.split('_')[0] in my_neighbor_id[0] or id.split('_')[0] in my_neighbor_id[1]:
            return idx+i
    return 999999

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

def getLanePosition(lanelets, l_id, link_idx):
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

def get_crosswalk_ids_points(lanelets, surfacemarks, remaining_global_ids, cur_id_idx): 
    selected_crosswalk_ids_points = []
    roi_waypoints = lanelets[remaining_global_ids[0]]['waypoints'][cur_id_idx:]
    if len(roi_waypoints) > 1: # string으로 만드려면 2개이상 점 필요
        roi_string = sh.LineString(roi_waypoints)
        
        # 현재 id의 crosswalkID
        # my_lane_crosswalkIds = list(set(lanelets[remaining_global_ids[0]]['crosswalkID'])) # 중복제거하면 순서정보가 없어셔서 안됨
        my_lane_crosswalkIds = lanelets[remaining_global_ids[0]]['crosswalkID']
        for id_ in my_lane_crosswalkIds: 
            crosswalk_polygon = sh.Polygon(surfacemarks[id_])
            if roi_string.intersects(crosswalk_polygon):
                selected_crosswalk_ids_points.append((id_, surfacemarks[id_]))
                # print("Crosswalk ID for MY link: ", id_)
        
    # 다음 id의 crosswalk
    if len(remaining_global_ids) > 2:
        # next_lane_crosswalkIds = list(set(lanelets[remaining_global_ids[1]]['crosswalkID'])) # 중복제거하면 순서정보가 없어셔서 안됨
        next_lane_crosswalkIds = lanelets[remaining_global_ids[1]]['crosswalkID']
        for id_ in next_lane_crosswalkIds: 
            selected_crosswalk_ids_points.append((id_, surfacemarks[id_]))
            # print("Crosswalk ID for NEXT link: ", id_)
        
    return selected_crosswalk_ids_points

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
            
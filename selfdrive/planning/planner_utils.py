import sys
import time
import math
import heapq as hq
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import Point as GPoint
import matplotlib.patches as patches
import pymap3d as pm

import cubic_spline_planner
from quadratic_spline_interpolate import QuadraticSplineInterpolate
from frenet_frame import frenet_optimal_planning
from viz import *  # 3

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)


def convert2enu(base, lat, lng):
        # lat0 = 35.64588122580907
        # lon0 = 128.40214778762413
        # h0 = 47.256
        x, y, _ = pm.geodetic2enu(lat, lng, 20, base[0], base[1], base[2])
        return [x,y]

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

def lanelet_matching_ref(tile, tile_size, t_pt, ref_path):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):

            selected_tile = tile.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    if id_ in ref_path:
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

def crosswalk_waypoint_matching(tile, tile_size, t_pt, crt_lane_id):
    crosswalk_waypoints = [
        ["185_3", "185_4", "185_5", "185_6", "185_7", "188_3", "188_4", "188_5", "188_6", "188_7", "191_26", "191_27", "191_28", "191_29", "191_30", "190_26", "190_27", "190_28", "190_29", "190_30"],
        ["193_3", "193_4", "193_5", "193_6", "193_7", "183_31", "183_32", "183_33", "183_34", "183_35"],
        ["190_3", "190_4", "190_5", "190_6", "190_7", "192_3", "192_4", "192_5", "192_6", "192_7", "186_26", "186_27", "186_28", "186_29", "186_30", "188_26", "188_27", "188_28", "188_29", "188_30"],
        ["184_4", "184_5", "184_6", "184_7", "184_8", "187_18", "187_19", "187_20", "187_21", "187_22"],

        ["198_3", "198_4", "198_5", "198_6", "198_7", "197_3", "197_4", "197_5", "197_6", "197_7", "195_27", "195_28", "195_29", "195_30", "195_31", "281_19", "281_20", "281_21", "281_22", "281_23"],
        ["200_4", "200_5", "200_6", "200_7", "200_8", "194_1", "194_2", "194_3", "194_4", "194_5"],
        ["205_3", "205_4", "205_5", "205_6", "205_7", "203_3", "203_4", "203_5", "203_6", "203_7", "200_27", "200_28", "200_29", "200_30", "200_31", "285_18", "285_19", "285_20", "285_21", "285_22"],
        ["194_3", "194_4", "194_5", "194_6", "194_7", "201_31", "201_32", "201_33", "201_34", "201_35"],

        ["211_3", "211_4", "211_5", "211_6", "211_7", "210_3", "210_4", "210_5", "210_6", "210_7", "212_27", "212_28", "212_29", "212_30", "212_31", "209_19", "209_20", "209_21", "209_22", "209_23"],
        ["208_3", "208_4", "208_5", "208_6", "208_7", "207_32", "207_33", "207_34", "207_35", "207_36"],
        ["207_3", "207_4", "207_5", "207_6", "207_7", "208_33", "208_34", "208_35", "208_36", "208_37"],

        ["181_17", "181_18", "181_19", "181_20", "181_21", "176_3", "176_4", "176_5", "176_6", "176_7"],
        ["179_24", "179_25", "179_26", "179_27", "179_28", "178_2", "178_3", "178_4", "178_5", "178_6"],
        ["177_3", "177_4", "177_5", "177_6", "177_7"]]

    crosswalk_lane_id = [
        ["57", "75", "187", "188", "190", "191", "184", "279", "186", "185"],
        ["59", "182", "193", "279", "185", "183", "189"], 
        ["78", "39", "192", "191", "190", "189", "188", "186", "193", "280"],
        ["182", "187", "60", "280", "183", "184", "192"],

        ["79", "80", "198", "199", "196", "197", "195", "203", "205", "281"],
        ["77", "200", "201", "281", "194", "204", "197"],
        ["24", "81", "202", "203", "204", "205", "285", "198","196", "200"],
        ["199", "201", "285", "194", "195", "40", "202"],

        ["31", "82", "211", "210", "209", "212"],
        ["73", "208", "209", "207", "210"],
        ["211", "208", "25", "207", "212"],

        ["180", "181", "88", "176"],
        ["179", "84", "85", "177", "58"],
        ["87", "177"]]


    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float(1.6)
    l_id, l_idx = None, None

    crosswalk_temp = []

    for lane_id_idx in range(len(crosswalk_lane_id)):
        if crt_lane_id in crosswalk_lane_id[lane_id_idx]:
            crosswalk_temp = crosswalk_temp + crosswalk_waypoints[lane_id_idx]

    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tile.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            l_id = id_
                            l_idx = data['idx'][idx]
                            if str(l_id) + "_" + str(l_idx) in crosswalk_temp:
                                return True
    
    return False

def crosswalk_waypoint_matching2(tile, tile_size, t_pt):
    crosswalk_waypoints = ["185_3", "185_4", "185_5", "185_6", "185_7", "188_3", "188_4", "188_5", "188_6",
        "188_7", "191_26", "191_27", "191_28", "191_29", "191_30", "190_26", "190_27", "190_28", "190_29",
        "190_30", "193_3", "193_4", "193_5", "193_6", "193_7", "183_31", "183_32", "183_33", "183_34", "183_35",
        "190_3", "190_4", "190_5", "190_6", "190_7", "192_3", "192_4", "192_5", "192_6", "192_7", "186_26",
        "186_27", "186_28", "186_29", "186_30", "188_26", "188_27", "188_28", "188_29", "188_30", "184_4",
        "184_5", "184_6", "184_7", "184_8", "187_18", "187_19", "187_20", "187_21", "187_22", "198_3", "198_4",
        "198_5", "198_6", "198_7", "197_3", "197_4", "197_5", "197_6", "197_7", "195_27", "195_28", "195_29",
        "195_30", "195_31", "281_19", "281_20", "281_21", "281_22", "281_23", "200_4", "200_5", "200_6", "200_7",
        "200_8", "194_1", "194_2", "194_3", "194_4", "194_5", "205_3", "205_4", "205_5", "205_6", "205_7",
        "203_3", "203_4", "203_5", "203_6", "203_7", "200_27", "200_28", "200_29", "200_30", "200_31", "285_18",
        "285_19", "285_20", "285_21", "285_22", "194_3", "194_4", "194_5", "194_6", "194_7", "201_31", "201_32",
        "201_33", "201_34", "201_35", "211_3", "211_4", "211_5", "211_6", "211_7", "210_3", "210_4", "210_5",
        "210_6", "210_7", "212_27", "212_28", "212_29", "212_30", "212_31", "209_19", "209_20", "209_21", "209_22",
        "209_23", "208_3", "208_4", "208_5", "208_6", "208_7", "207_32", "207_33", "207_34", "207_35", "207_36",
        "207_3", "207_4", "207_5", "207_6", "207_7", "208_33", "208_34", "208_35", "208_36", "208_37", "181_17",
        "181_18", "181_19", "181_20", "181_21", "176_3", "176_4", "176_5", "176_6", "176_7", "179_24", "179_25",
        "179_26", "179_27", "179_28", "178_2", "178_3", "178_4", "178_5", "178_6", "177_3", "177_4", "177_5",
        "177_6", "177_7"]


    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float(1.6)
    l_id, l_idx = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tile.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            l_id = id_
                            l_idx = data['idx'][idx]
                            if str(l_id) + "_" + str(l_idx) in crosswalk_waypoints:
                                return True
    
    return False

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


def route_plan(lanelets, graph, e_id, e_idx, g_id, g_idx, pub_search):
    def idn2node(id_, n):
        return '%s_%s' % (id_, n)

    def node2id(node):
        id_, n = node.split('_')
        return id_, int(n)

    e_node = idn2node(e_id, e_idx)
    g_node = idn2node(g_id, g_idx)

    print(dijkstra(graph, e_node, g_node))
    exit()

    distances = {}
    previous = {}
    nodes = []
    closed = []

    d_thres = 20.0

    distances[e_node] = 0.0
    hq.heappush(nodes, [0.0, e_node])
    closed.append(e_id)
    import time
    while nodes:
        c_node = hq.heappop(nodes)[1]
        a, b = node2id(c_node)
        pub_search.publish(ObjectsViz([lanelets[a]['waypoints'][b]]))
        time.sleep(0.05)
        if c_node.split('_')[0] == g_id.split('_')[0]:
            print('arrived!')

            path = []

            while previous.get(c_node) is not None:
                path.append(c_node)
                c_node = previous[c_node]
                print(c_node)
            path.reverse()
            return (path, 0.0)

        for n_node, dir_ in graph[c_node].items():
            if n_node not in closed:
                if dir_ == 0:  # straight
                    distances[n_node] = distances[c_node] + 1.0
                    hq.heappush(nodes, [distances[n_node], n_node])
                    previous[n_node] = c_node
                    closed.append(n_node)

                elif dir_ == -1:  # left
                    dist = 0
                    t_node = n_node
                    done = False
                    while t_node is not None:
                        if done or len(graph[t_node].items()) == 0:
                            break

                        for n, (f_node, dir_) in enumerate(graph[t_node].items()):
                            if n == 0:
                                t_node = None

                            if dir_ == 0:
                                t_node = f_node
                                dist += 1.0
                                if dist > d_thres:
                                    distances[f_node] = distances[c_node] + dist
                                    hq.heappush(
                                        nodes, [distances[f_node], f_node])
                                    previous[f_node] = c_node
                                    closed.append(f_node)
                                    done = True

                else:  # right
                    dist = 0
                    t_node = n_node
                    done = False
                    while t_node is not None:
                        if done or len(graph[t_node].items()) == 0:
                            break

                        for n, (f_node, dir_) in enumerate(graph[t_node].items()):
                            if n == 0:
                                t_node = None

                            if dir_ == 0:
                                t_node = f_node
                                dist += 1.0
                                if dist > d_thres:
                                    distances[f_node] = distances[c_node] + dist
                                    hq.heappush(
                                        nodes, [distances[f_node], f_node])
                                    previous[f_node] = c_node
                                    closed.append(f_node)
                                    done = True

    return None


def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points


def modify_graph(lanelets, tmap, mlgraph, const_sites):
    graph = mlgraph.graph
    modified_graph = copy.copy(graph)
    reversed_graph = mlgraph.reversed_graph
    tiles = tmap.tiles
    tile_size = tmap.tile_size

    for points in const_sites:
        polygon = Polygon(points)
        pre_point = None
        tile_ids = []
        for point in points:
            if pre_point is None:
                pre_point = point
            else:
                wx, wy = zip(*[pre_point, point])
                itp = QuadraticSplineInterpolate(list(wx), list(wy))
                for n, ds in enumerate(np.arange(0.0, itp.s[-1], 1.0)):
                    x, y = itp.calc_position(ds)
                    tile_id = int(x // tile_size), int(y // tile_size)
                    if tile_id not in tile_ids:
                        tile_ids.append(tile_id)

        nodes_in_construction_site = []
        for tile_id in tile_ids:
            tile = tiles.get(tile_id)
            if tile is not None:
                for id_, data in tile.items():
                    for n, pt in enumerate(data['waypoints']):
                        point = GPoint(pt[0], pt[1])
                        if point.within(polygon):
                            node_id = node_matching(
                                lanelets, id_, data['idx'][n])
                            if node_id not in nodes_in_construction_site:
                                nodes_in_construction_site.append(node_id)

        for node_id in nodes_in_construction_site:
            modified_graph[node_id] = {}

            if reversed_graph.get(node_id) is not None:
                for id_ in reversed_graph[node_id]:
                    if modified_graph[id_].get(node_id) is not None:
                        del modified_graph[id_][node_id]

    return modified_graph


def ref_interpolation(points, precision, min_v, ref_v):
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

# def get_sequence(lanelets, c_id):
#     sequence = []
#     next_id = c_id
#     while next_id is not None:
#         sequence.append(next_id)
#         successor = lanelets[next_id]['successor']
#         if len(successor) == 1:
#             s_id = successor[0]
#             if not lanelets[s_id]['intersection'] and not lanelets[s_id]['uTurn']:
#                 next_id = s_id
#             else:
#                 next_id = None
#         else:
#             next_id = None

#     return sequence

# def get_node_list(lanelets, c_id):
#     sequence_list = []
#     sequence = get_sequence(lanelets, c_id)
#     open_list = [sequence]
#     closed = []

#     while len(open_list) > 0:
#         sequence = open_list.pop()
#         last_id = sequence[-1]
#         if last_id not in closed:
#             successor = lanelets[last_id]['successor']
#             if len(successor) > 1:
#                 for s_id in successor:
#                     extend_sequence = get_sequence(lanelets, s_id)
#                     open_list.append(sequence + extend_sequence)
#             else:
#                 sequence_list.append(sequence)
#                 closed.extend(sequence)

#     return sequence_list

# def route_plan(lanelets, graph, e_id, e_idx, g_id, g_idx, pub_search):
#     closed = []
#     nodes = []
#     distance = {}
#     previous = {}

#     e_node_list = get_node_list(lanelets, e_id)

#     for e_node in e_node_list:
#         closed.extend(e_node)
#         distance[str(e_node)] = 0.0
#         hq.heappush(nodes, [distance[str(e_node)], e_node])

#     wp = []
#     while nodes:
#         c_node = hq.heappop(nodes)[1]
#         if g_id in c_node:
#             path = []

#             while previous.get(str(c_node)) is not None:
#                 path.append(c_node)
#                 for id_ in c_node:
#                     wp.extend(lanelets[id_]['waypoints'])

#                 c_node = previous.get(str(c_node))

#             for id_ in e_node:
#                 wp.extend(lanelets[id_]['waypoints'])

#             path.append(e_node)
#             path.reverse()
#             pub_search.publish(ObjectsViz(wp))
#             print(path)
#             # exit()
#             return path

#         for id_ in c_node:
#             wp.extend(lanelets[id_]['waypoints'])

#         last_id = c_node[-1]
#         for s_id in lanelets[last_id]['successor']:
#             if s_id not in closed:
#                 s_node_list = get_node_list(lanelets, s_id)
#                 for s_node in s_node_list:
#                     closed.extend(s_node)
#                     previous[str(s_node)] = c_node
#                     distance[str(s_node)] = distance[str(c_node)] + 1.0
#                     hq.heappush(nodes, [distance[str(s_node)], s_node])

#         for c_id in c_node:
#             left_id = lanelets[c_id]['adjacentLeft']
#             if left_id is not None and left_id not in closed:
#                 left_node_list = get_node_list(lanelets, left_id)
#                 for left_node in left_node_list:
#                     closed.extend(left_node)
#                     previous[str(left_node)] = c_node
#                     distance[str(left_node)] = distance[str(c_node)] + 1.0
#                     hq.heappush(nodes, [distance[str(left_node)], left_node])

#             right_id = lanelets[c_id]['adjacentRight']
#             if right_id is not None and right_id not in closed:
#                 right_node_list = get_node_list(lanelets, right_id)
#                 for right_node in right_node_list:
#                     closed.extend(right_node)
#                     previous[str(right_node)] = c_node
#                     distance[str(right_node)] = distance[str(c_node)] + 1.0
#                     hq.heappush(nodes, [distance[str(right_node)], right_node])
#     return None

def node_to_waypoints(lanelet, shortest_path):
    final_path = []
    
    for id in shortest_path:
        split_id = id.split('_')
        if len(split_id) == 2:
            s_idx, e_idx = (lanelet[split_id[0]]['cut_idx'][int(split_id[1])])
            final_path.append(lanelet[split_id[0]]['waypoints'][int((s_idx+e_idx)//2)])
        else:
            final_path.extend(lanelet[split_id[0]]['waypoints'])
    return final_path

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

    
def node_to_lanelet(shortest_path):
    global_path = []
    for id_ in shortest_path:
        t_id = id_.split('_')[0]
        if len(global_path) == 0 or global_path[-1] != t_id:
            global_path.append(t_id)
    return global_path


def path_to_group(lanelet, G, global_path):
    selected_group_list = []
    selected_group = []

    closed = []
    pre_group = None
    for id_ in global_path:
        group_n = lanelet[id_].get('group')

        if group_n is not None:
            temp_group = G[group_n]
        else:
            temp_group = [id_]

        if pre_group != temp_group:
            if temp_group in closed:
                selected_group_list.append(selected_group)
                if len(selected_group) >= 2:
                    selected_group = selected_group[-2:]
                else:
                    selected_group = selected_group[-1:]
                closed = []

            selected_group.append(temp_group)
            closed.append(temp_group)

        pre_group = temp_group

    selected_group_list.append(selected_group)

    return selected_group_list


def generate_ref_paths(lanelet, G, precision, lc_cost, v_offset, min_v, ref_v, ego_lanelet, goal_lanelet, selected_group, selected_group_n, selected_group_len):
    goal_id, goal_idx = goal_lanelet
    goal_group = []

    ref_paths = {}
    id2ref = {}
    n = 0

    if len(selected_group) == 1:
        group = selected_group[0]
        for j, id_ in enumerate(group):
            if id2ref.get(id_) is None:
                ref_paths[n] = [id_]
                id2ref[id_] = n
                if id_ == goal_id:
                    goal_group = [n, j, group]
                n += 1
    else:
        for i, group in enumerate(selected_group):
            if i != len(selected_group) - 1:
                next_group = selected_group[i+1]
                for j, id_ in enumerate(group):
                    if id2ref.get(id_) is None:
                        ref_paths[n] = [id_]
                        id2ref[id_] = n
                        if id_ == goal_id:
                            goal_group = [n, j, group]
                        n += 1

                    if not lanelet[id_]['disappear']:
                        for k, successor in enumerate(lanelet[id_]['successor']):
                            if successor in next_group:
                                if not lanelet[successor]['appear']:
                                    ref_n = id2ref[id_]
                                    id2ref[successor] = ref_n
                                    ref_paths[ref_n].append(successor)

                                    if successor == goal_id:
                                        for kk, ii in enumerate(next_group):
                                            if ii == goal_id:
                                                goal_group = [
                                                    ref_n, kk, next_group]
            else:
                if len(goal_group) == 0:
                    for j, id_ in enumerate(group):
                        if id2ref.get(id_) is None:
                            ref_paths[n] = [id_]
                            id2ref[id_] = n
                            if id_ == goal_id:
                                goal_group = [n, j, group]
                            n += 1

    waypoints = {}
    offset = {}
    max_v = {}

    if selected_group_n == selected_group_len - 1:
        offset[goal_group[0]] = lanelet[goal_id]['idx_num'] - goal_idx
        t = goal_group[1]

        for i, g_id in enumerate(goal_group[2]):
            ref_n = id2ref.get(g_id)
            if g_id != goal_id:
                offset[ref_n] = offset[goal_group[0]] + \
                    abs(t - i) * int(lc_cost / precision / 2)

    for i, ref_path in ref_paths.items():
        id_ = ref_path[-1]
        waypoints[i] = []
        max_v[i] = []
        for ref_id in ref_path:
            waypoints[i].extend(lanelet[ref_id]['waypoints'])
            max_v[i].extend(ref_to_max_v(lanelet[ref_id]['waypoints'], precision,
                            v_offset, min_v, ref_v, lanelet[ref_id]['speedLimit']))

        group_n = lanelet[id_].get('group')
        if group_n is not None:
            if selected_group_n != selected_group_len - 1 or id_ not in goal_group[2]:
                t = 0
                offset_ = 0
                for n, g_id in enumerate(G[group_n]):
                    ref_n = id2ref.get(g_id)
                    if g_id == id_:
                        t = n
                        t_ref_n = ref_n
                    else:
                        if g_id != ref_paths[ref_n][-1]:
                            offset_ = n

                if selected_group_n == selected_group_len - 1 or G[group_n] != selected_group[-1]:
                    offset[t_ref_n] = abs(offset_ - t) * \
                        int(lc_cost / precision / 2)

    for i, ref_path in ref_paths.items():
        waypoints[i] = []
        max_v[i] = []
        for ref_id in ref_path:
            waypoints[i].extend(lanelet[ref_id]['waypoints'])
            max_v[i].extend(ref_to_max_v(lanelet[ref_id]['waypoints'], precision,
                            v_offset, min_v, ref_v, lanelet[ref_id]['speedLimit']))

        offset_ = offset.get(i)
        if offset_ is not None:
            if selected_group_n == 0 and ref_id == ego_lanelet[0]:
                if len(waypoints[i]) - offset_ < ego_lanelet[1]:
                    continue

            waypoints[i] = waypoints[i][:-offset_]
            max_v[i] = max_v[i][:-offset_]

    return [ref_paths, waypoints, id2ref, max_v]


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


def get_objects_s(pts, objects, precision, thres):
    objects_s = []
    filtered_objects = []

    for pt in objects:
        cte, idx = calc_cte_and_idx(pts, pt)
        s = idx * precision
        if abs(cte) <= thres:
            objects_s.append(s)
            filtered_objects.append(pt)

    return objects_s, filtered_objects


def select_ref_path(lanelet, cur_n, ref_paths_data, ego_lanelet, pt, objects):
    e_id, e_idx = ego_lanelet

    ref_paths = ref_paths_data[0]
    waypoints = ref_paths_data[1]
    id2ref = ref_paths_data[2]

    cand = []
    cost = []

    cur_idx = find_nearest_idx(waypoints[cur_n], pt)
    cur_rdist = len(waypoints[cur_n]) - cur_idx
    cost.append(1.0 / cur_rdist)
    cand.append(['current', cur_n])

    left_id = lanelet[e_id]['adjacentLeft']
    if left_id is not None:
        left_n = id2ref.get(left_id)
        if left_n is not None:
            left_idx = find_nearest_idx(waypoints[left_n], pt)
            left_rdist = len(waypoints[left_n]) - left_idx
            if left_rdist > 0.0:
                cost.append(1.0 / max(abs(left_rdist - 5.0), 10.0))
                cand.append(['left', left_n])

    right_id = lanelet[e_id]['adjacentRight']
    if right_id is not None:
        right_n = id2ref.get(right_id)
        if right_n is not None:
            right_idx = find_nearest_idx(waypoints[right_n], pt)
            right_rdist = len(waypoints[right_n]) - right_idx
            if right_rdist > 0.0:
                cost.append(1.0 / max(abs(right_rdist - 5.0), 10.0))
                cand.append(['right', right_n])

    min_idx = np.argmin(cost)

    return cand[min_idx]


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


def generate_local_path(csp, pt, v, s, cte):
    c_speed = v  # current speed [m/s]
    c_d = cte  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = s

    local_paths, selected = frenet_optimal_planning(
        csp, s0, c_speed, c_d, c_d_d, c_d_dd)

    return local_paths, selected


def filter_traffic_data(lanelet, precision, st_param, traffic_light, ref_path, ref_path_wp):
    t_max = st_param['t_max']
    traffic_data = []

    for tl_id, data in traffic_light.items():
        tl_state = data['state']
        tl_time = data['time']
        # tl_left_state = data['left_state']
        # tl_left_time = data['left_time']
        for id_ in ref_path:
            if tl_id in lanelet[id_]['trafficLight']:
                tl_idx = find_nearest_idx(
                    ref_path_wp, lanelet[id_]['waypoints'][-1])
                tl_s = tl_idx * precision

                if tl_state == 'green':
                    tl_time = [tl_time, t_max]
                elif tl_state == 'yellow':
                    tl_time = [0, t_max]
                elif tl_state == 'red':
                    tl_time = [0, tl_time]

                traffic_data.append([tl_s, tl_time, tl_state])

    return traffic_data


def lane_change_available(lanelet, ego_lanelet, change_dir, objects):
    return True


def velocity_plan(ax, drawn, st_param, ref_v, last_s, s, max_v, cur_v, cur_a, tl_s, tl_time, tl_state, objects_s): #, objects_s=None, rel_v=None):
    #cur_v : m/s
    s_min, s_max, t_max = st_param['s_min'], st_param['s_max'], st_param['t_max']
    dt, dt_exp = st_param['dt'], st_param['dt_exp']
    ref_v = ref_v * KPH_TO_MPS
    ds_exp = ref_v * dt_exp

    target_v = 0.0
    s += 3.0

    if drawn is not None:
        for d in drawn:
            d.remove()

    drawn = []
    obstacles = []

    last = [(0.0, last_s-2.0), (t_max, last_s-2.0),(t_max, last_s+5.0), (0.0, last_s+5.0)]
    obstacles.append(last)
    d = ax.add_patch(patches.Polygon([pt for pt in last], closed=True, edgecolor='black', facecolor='gray'))
    drawn.append(d)


    tl_offset = 10
    #remove tl_state == 'green' or 'yellow' or 'red' ( recog by string )
    # green = 2, yellow = 1, red = 0 
    # red = 1, green = 2, yellow = 3
    traffic_light = []
    # print("state:", tl_state,"time:",tl_time, "s", tl_s)
    if tl_state == 1: #red
        light_time = min(t_max-3.0, tl_time/10)
        if cur_v > 15 * KPH_TO_MPS:
            traffic_light = [(0, tl_s), (5.0,tl_s), (5.0, tl_s+tl_offset), (0, tl_s+tl_offset)]
        else:
            traffic_light = [(0, tl_s), (light_time, tl_s), (light_time, tl_s+tl_offset), (0, tl_s+tl_offset)]
        obstacles.append(traffic_light)
        d = ax.add_patch(patches.Polygon([pt for pt in traffic_light], closed=True, edgecolor='black', facecolor='red'))
        drawn.append(d)
    elif tl_state == 2: #green
        light_time = tl_time/10
        if light_time < 2 :
            traffic_light = [(0, tl_s), (4.0,tl_s), (4.0, tl_s+tl_offset), (0, tl_s+tl_offset)]
            obstacles.append(traffic_light)
            d = ax.add_patch(patches.Polygon([pt for pt in traffic_light], closed=True, edgecolor='black', facecolor='green'))
            drawn.append(d)
    elif tl_state == 3: #yellow
        traffic_light= [(0, tl_s), (5.0,tl_s), (5.0, tl_s+tl_offset), (0, tl_s+tl_offset)]
        obstacles.append(traffic_light)
        d = ax.add_patch(patches.Polygon([pt for pt in traffic_light], closed=True, edgecolor='black', facecolor='yellow'))
        drawn.append(d)
    
    #[OLD] Object Detection control 
    obj_offset = 5
    if cur_v < 30 * KPH_TO_MPS:
        obj_time = 2
    elif cur_v < 40* KPH_TO_MPS:
        obj_time = 3
    elif cur_v < 100* KPH_TO_MPS:
        obj_time = 3.5 
    for os in objects_s:
        if os[1] > -2.0 and os[1] < 2.0:
            obj = [(0.0, objects_s-obj_offset), (obj_time, objects_s-obj_offset),
                    (obj_time, objects_s+obj_offset), (0.0, objects_s+obj_offset)]
            obstacles.append(obj)
            d = ax.add_patch(patches.Polygon([pt for pt in obj], closed=True, edgecolor='black', facecolor='blue'))
            drawn.append(d)

    ax.set_ylim([s+s_min, s+s_max])
    ax.set_yticks([i for i in range(int(s+s_min), int(s+s_max))])

    #[OLD] Traffic Light control
    '''
    if tl_state == 2:
        traffic_light = [(0, tl_s-tl_offset), (tl_time[0],
                                                tl_s-tl_offset), (tl_time[0], tl_s), (0, tl_s)]
        obstacles.append(traffic_light)
        d = ax.add_patch(patches.Polygon(
            [pt for pt in traffic_light], closed=True, edgecolor='black', facecolor='green'))
        drawn.append(d)
    elif tl_state == 1:
        obstacles.append(traffic_light)
        d = ax.add_patch(patches.Polygon(
            [pt for pt in traffic_light], closed=True, edgecolor='black', facecolor='yellow'))
        drawn.append(d)
    elif tl_state == 0:
        obstacles.append(traffic_light)
        d = ax.add_patch(patches.Polygon(
            [pt for pt in traffic_light], closed=True, edgecolor='black', facecolor='red'))
        drawn.append(d)
    '''
    


    ############### Planning Start ###############
    start = (0.0, s, cur_v, cur_a)  # t, s, v, a
    start_time = time.time()
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

            d = ax.plot(t_list, s_list, '-m')
            drawn.append(d[0])
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

            d = ax.plot([chosen_c_node[0], neighbor_t], [
                        chosen_c_node[1], neighbor_s], '-c')
            drawn.append(d[0])

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

    return target_v, drawn


def local_to_global(local_data, pos):
    x, y, yaw = pos
    yaw = math.radians(yaw)

    local_data = np.hstack((local_data, np.zeros((local_data.shape[0], 1))))
    local_data = np.hstack((local_data, np.ones((local_data.shape[0], 1))))

    n = 1
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    M = np.zeros((n, 4, 4))
    M[:, 0, 0] = cos_yaw
    M[:, 0, 1] = -sin_yaw
    M[:, 1, 0] = sin_yaw
    M[:, 1, 1] = cos_yaw
    M[:, 2, 2] = 1.0
    M[:, 3, 3] = 1.0
    M[:, 0, 3] = x
    M[:, 1, 3] = y

    global_data = np.dot(M, local_data.T).transpose(0, 2, 1)[:, :, 0:2]

    return global_data

change = 0 
# ["27_9", "34_9", "42_9", "61_10", "80_10", "83_9", "84_9", "85_9", "86", "87", "93", "94"]
def siginal_light_toggle(path, ego_idx, precision, t_map, lmap, stage):
    ids = []
    try:
        forward_range = 40 # meter
        ego_pt = path[ego_idx]
        ego_lanelet_id, _ = lanelet_matching(t_map.tiles, t_map.tile_size, ego_pt)
        left_lanelet = lmap.lanelets[ego_lanelet_id]['adjacentLeft']
        right_lanelet = lmap.lanelets[ego_lanelet_id]['adjacentRight']

        for distance in range(forward_range+1):
            forward_pt = path[ego_idx+int(distance/precision)]
            id, _ = lanelet_matching(t_map.tiles, t_map.tile_size, forward_pt)
            ids.append(id)

        forward_lanelet_ids = np.array(ids)
        # 0: normal, 1: left, 2: right
        # if forward_lanelet_id == left_lanelet or (stage == 1 and ):
        print(forward_lanelet_ids)
        if np.any(forward_lanelet_ids == left_lanelet):
            change = 1
        elif np.any(forward_lanelet_ids == right_lanelet):
            change = 2
        else:
            change = 0
        print(change)
        return change
    except IndexError:
        pass

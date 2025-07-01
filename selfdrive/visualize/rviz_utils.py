import os
import tf
import math
import numpy as np
import time
import pickle

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from itertools import chain
from collections import defaultdict
from selfdrive.visualize.libs.quadratic_spline_interpolate import QuadraticSplineInterpolate

dir_path = os.path.dirname(os.path.realpath(__file__))

prev_marker_count = 0
prev_text_count = 0

def ObjectsViz(objects):
    global prev_marker_count
    global prev_text_count
    text_count=0

    array = MarkerArray()
    marker = Marker()
    textarray = MarkerArray()
    text = Marker()

    for n, pt in enumerate(objects):
        if 0 < pt[2] < 200 and -1.45<pt[3]<1.45:
            color = (1.0, 0.0, 0.0, 1.0)
            s = str(pt[2] / 2) if pt[2] else 0
            d = str(round(pt[3], 1)) if pt[3] else 0
            v = str(round(max(pt[5], 0))) if pt[5] and not math.isnan(pt[5]) else 0
            # message = "s:{}m d:{}m v:{}km/h".format(s, d, v)
            message = "v:{}km/h".format(v)
            text = Text('obstacle_information', text_count, 1.0, (1, 1, 1, 1), message)
            text.pose.position = Point(x=pt[0], y=pt[1], z=3.0)
            text_count +=1
            textarray.markers.append(text)
        elif -100 < pt[2] < 100 and (-4.05 < pt[3] < -1.45 or 1.45 < pt[3] < 4.05):
            color = (1.0, 1.0, 0.0, 1.0)
        else:
            color = (0.0, 1.0, 0.0, 1.0)
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(pt[4]))
        # marker = Sphere('obstacle', n, (round(pt[0],1), round(pt[1],1)), 2.1, color)
        marker = Points('obstacle', n, 1.0, color)
        marker.points.append(Point(x=pt[0], y=pt[1], z=1.0))
        array.markers.append(marker)

    if len(objects) < prev_marker_count:
        for i in range(len(objects), prev_marker_count):
            marker = Marker()
            marker.ns = "obstacle"
            marker.header.frame_id = "world"
            marker.id = i
            marker.action = Marker.DELETE
            array.markers.append(marker)

    if text_count < prev_text_count:
        for i in range(text_count, prev_text_count):
            text = Marker()
            text.ns = "obstacle_information"
            text.header.frame_id = "world"
            text.id = i
            text.action = Marker.DELETE
            textarray.markers.append(text)

    prev_marker_count = len(objects)
    prev_text_count = text_count
    return array, textarray


def TrafficLightViz(tl, cls):
    quaternion = tf.transformations.quaternion_from_euler(
        0.0, 0.0, 0.0)
    if cls == 0:  # Red
        color = (1.0, 0.0, 0.0, 1.0)
    elif cls == 1:  # Yellow
        color = (1.0, 1.0, 0.0, 1.0)
    elif cls == 2:  # Green
        color = (0.0, 1.0, 0.0, 1.0)
    marker = Cube('traffic light', 0, 1, quaternion, color)
    marker.pose.position = Point(x=tl[0], y=tl[1], z=tl[2])
    return marker


def LookAheadViz(pt):
    data = (pt[0], pt[1])
    marker = Sphere('look_ahead', 0, data, 2.0, (0.0, 0.0, 1.0, 1.0))
    return marker

def GlobalPathViz(waypoints):
    marker = Line('global_path', 999, 0.1, (0.0, 1.0, 0.0, 1.0))
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.1))
    return marker

def LocalPathViz(waypoints):
    marker = Line('local_path', 999, 0.2, (1.0, 0.0, 0.0, 1.0))
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.4))
    return marker

def ForwardPathViz(waypoints):
    marker = Line('forward_path', 999, 0.2, (1.0, 0.0, 1.0, 1.0))
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.2))
    return marker

def CrosswalkViz(waypoints):
    marker = Line('crosswalk', 999, 0.4, (1.0, 0.2, 0.6, 1.0))
    for _, pt in enumerate(waypoints):
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.2))
    return marker

def StopLineViz(waypoints):
    marker = Line('stopline', 999, 0.4, (0.0, 0.0, 1.0, 1.0))
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.3))
    return marker

    # marker = Line('stopline', 999, 0.4, (1.0, 1.0, 0.0, 1.0))
    # for _, pt in enumerate(waypoints):
    #     marker.points.append(Point(x=pt[0], y=pt[1], z=0.2))
    # return marker


def GoalViz(pt):
    marker = Text('goal', 0, 2.0, (1.0, 1.0, 1.0, 1.0), 'GOAL')
    marker.pose.position = Point(x=pt[0], y=pt[1], z=1.0)
    return marker


# map이 songdo 아닐 때
def LaneletMapViz(lanelet, for_viz):
    array = MarkerArray()
    for id_, data in lanelet.items():
        for n, (leftBound, leftType) in enumerate(zip(data['leftBound'], data['leftType'])):
            marker = Bound('leftBound', id_, n, leftBound,
                           leftType, (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

        for n, (rightBound, rightType) in enumerate(zip(data['rightBound'], data['rightType'])):
            marker = Bound('rightBound', id_, n, rightBound,
                           rightType, (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

        # color = (random.randint(0, 255)/255.0, random.randint(0,
        #          255)/255.0, random.randint(0, 255)/255.0, 1.0)

        # marker = Waypoints(id_, data['waypoints'], color)
        # array.markers.append(marker)

        # idx = len(data['waypoints']) // 2
        # mid_pt = data['waypoints'][idx]

        # marker = ID(id_, mid_pt, color)
        # array.markers.append(marker)

    for n, (points, type_) in enumerate(for_viz):  
        if type_ == 'stop_line':
            marker = Bound('for_viz', n, n, points,
                           'solid', (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)
        else:
            marker = Bound('for_viz', n, n, points,
                           type_, (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

    return array

# map이 songdo일 때
def VectorMapVis(map_data):
    start = time.time()
    lanelet = map_data['lanelets']
    #side_lanelets = map_data['side_lanelets']
    stoplines = map_data['stoplines']
    safetysigns = map_data['safetysigns']
    surfacemarks = map_data['surfacemarks']
    trafficlights = map_data['trafficlights']
    vehicleprotectionsafetys = map_data['vehicleprotectionsafetys']
    #speedbumps = map_data['speedbumps']
    postpoints = map_data['postpoints']

    array = MarkerArray()
    for id_, data in lanelet.items():
        for n, (leftBound, leftType) in enumerate(zip(data['leftBound'], data['leftType'])):
            marker = Bound('leftBound', id_, n, leftBound,
                           leftType, (1.0, 1.0, 1.0, 0.5))
            array.markers.append(marker)

        for n, (rightBound, rightType) in enumerate(zip(data['rightBound'], data['rightType'])):
            marker = Bound('rightBound', id_, n, rightBound,
                           rightType, (1.0, 1.0, 1.0, 0.5))
            array.markers.append(marker)

    # for id_, data in side_lanelets.items():
    #     marker = Bound('side', id_, 0, data[0], data[1], (1.0, 1.0, 1.0, 0.5))
    #     array.markers.append(marker)

    for id_, data in safetysigns.items():
        marker = Bound('safetysign', id_, n, data,
                       'solid', (1.0, 1.0, 1.0, 0.5))
        array.markers.append(marker)

    for id_, data in stoplines.items():
        marker = Bound('stopline', id_, 0, data, 'solid', (1.0, 1.0, 1.0, 0.5))
        array.markers.append(marker)

    for id_, data in surfacemarks.items():
        marker = Bound('surfacemark', id_, 0, data,
                       'solid', (1.0, 1.0, 1.0, 0.5))
        array.markers.append(marker)

    for id_, data in trafficlights.items():
        # marker = Sphere('traifficlight_%s' %
        #                 (id_), 0, data, 0.1, (1.0, 1.0, 1.0, 0.5))
        marker = Points('trafficlight_%s' % id_, 0, 0.1, (1.0, 1.0, 1.0, 0.5))
        marker.points.append(Point(x=data[0], y=data[1], z=1.0))
        array.markers.append(marker)

    for id_, data in vehicleprotectionsafetys.items():
        marker = Bound('vehicleprotectionsafety', id_, 0,
                       data, 'solid', (1.0, 1.0, 1.0, 0.5))
        array.markers.append(marker)

    # for id_, data in speedbumps.items():
    #     marker = Bound('speedbump', id_, 0, data,
    #                    'solid', (1.0, 1.0, 1.0, 0.5))
    #     array.markers.append(marker)

    for id_, data in postpoints.items():
        # marker = PostPoint('postpoint_%s' % (id_), 0, data,
        #                    0.2, 4.0, (1.0, 1.0, 1.0, 0.5))
        marker = Points('postpoint_%s' % id_, 0, 0.2, (1.0, 1.0, 1.0, 0.5))
        marker.points.append(Point(x=data[0], y=data[1], z=2+data[2]))
        array.markers.append(marker)
    
    print(f"<Elapsed:Lanelet> Converting into LANELET: {time.time() - start}")
    return array


def PostPoint(ns, id_, data, radius, height, color):
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.header.frame_id = 'map'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = radius
    marker.scale.y = radius
    marker.scale.z = height
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.position.x = data[0]
    marker.pose.position.y = data[1]
    marker.pose.position.z = height / 2.0 + data[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker


def MicroLaneletGraphViz_pickle(lanelet: dict,
                          graph: dict,
                          map_name: str = 'default') -> MarkerArray:
    base_dir   = os.path.dirname(os.path.abspath(__file__))
    pickle_dir = os.path.join(base_dir, 'pickles')
    os.makedirs(pickle_dir, exist_ok=True)

    pkl_path = os.path.join(pickle_dir, f'{map_name}_lanelet_graph.pkl')

    if os.path.isfile(pkl_path):
        try:
            with open(pkl_path, 'rb') as f:
                cached = pickle.load(f)
            if isinstance(cached, MarkerArray):
                return cached
        except Exception:
            pass  

    array = MarkerArray()

    for n, (node_id, data) in enumerate(graph.items()):
        split = node_id.split('_')

        if len(split) == 1:
            id_ = split[0]
            from_idx = lanelet[id_]['idx_num'] // 2
            from_pts = lanelet[id_]['waypoints']
            array.markers.append(Node(node_id, n, from_pts[from_idx],
                                      (1.0, 1.0, 1.0, 1.0)))

            for m, tgt_id in enumerate(data):
                t_split = tgt_id.split('_')
                pts = []

                if len(t_split) == 1:
                    tgt = t_split[0]
                    to_pts = lanelet[tgt]['waypoints']
                    to_idx = lanelet[tgt]['idx_num'] // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])
                else:
                    tgt, cut = t_split[0], int(t_split[1])
                    to_pts = lanelet[tgt]['waypoints']
                    to_idx = sum(lanelet[tgt]['cut_idx'][cut]) // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])

                array.markers.extend(
                    Edge(n*100000+m, pts, (0.0, 1.0, 0.0, 0.5)))

        else:
            id_, cut_n = split[0], int(split[1])
            from_idx = sum(lanelet[id_]['cut_idx'][cut_n]) // 2
            from_pts = lanelet[id_]['waypoints']
            array.markers.append(Node(node_id, n, from_pts[from_idx],
                                      (1.0, 1.0, 1.0, 1.0)))

            for m, tgt_id in enumerate(data):
                t_split = tgt_id.split('_')
                pts = []

                if len(t_split) == 1:
                    tgt = t_split[0]
                    to_pts = lanelet[tgt]['waypoints']
                    to_idx = lanelet[tgt]['idx_num'] // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])
                else:
                    tgt, cut = t_split[0], int(t_split[1])
                    to_pts = lanelet[tgt]['waypoints']
                    to_idx = sum(lanelet[tgt]['cut_idx'][cut]) // 2
                    pts = [from_pts[from_idx], to_pts[to_idx]]

                array.markers.extend(
                    Edge(n*100000+m, pts, (0.0, 1.0, 0.0, 0.5)))

    try:
        with pkl_path.open('wb') as f:
            pickle.dump(array, f, protocol=pickle.HIGHEST_PROTOCOL)
    except Exception as e:
        print(f'[MicroLaneletGraphViz2] 캐시 저장 실패: {e}')

    return array

# ──────────────────────────────
def MicroLaneletGraphViz(lanelet: dict, graph: dict) -> MarkerArray:
    def _as_list(pt):
        """NumPy 배열이면 tolist(), 이미 list/tuple이면 그대로 list()"""
        if isinstance(pt, np.ndarray):
            return pt.tolist()
        elif isinstance(pt, (list, tuple)):
            return list(pt)
        else:                             # geometry_msgs/Point 등
            return [pt.x, pt.y, pt.z]
    """
    lanelet[id] = {
        'waypoints': list(Point) | list[list] | np.ndarray(N,3),
        'idx_num'  : int,
        'cut_idx'  : [[s,e], ...]   # 그룹 lanelet만 존재
    }
    graph = { node_id: { target_id: cost, ... }, ... }
    """
    markers = MarkerArray()

    # 1) 전처리(최초 호출 시만 실질 변환)
    mid_idx, mid_pt, back_tail = {}, {}, {}
    seg_mid_idx, seg_front = defaultdict(dict), defaultdict(dict)

    for lid, data in lanelet.items():
        wp = data['waypoints']
        if not isinstance(wp, np.ndarray):
            wp = np.asarray([_as_list(p) for p in wp], dtype=np.float32)
            data['waypoints'] = wp     # 변환 결과 저장(한 번만)

        m = data['idx_num'] // 2
        mid_idx[lid] = m
        mid_pt[lid]  = wp[m]
        back_tail[lid] = wp[m:]       # view

        if 'cut_idx' in data:
            for n, (s, e) in enumerate(data['cut_idx']):
                sm = (s + e) // 2
                seg_mid_idx[lid][n] = sm
                seg_front[lid][n]   = wp[:sm]   # view

    # 2) 그래프 순회
    for n, (node_id, edges) in enumerate(graph.items()):
        base_id, *seg_part = node_id.split('_')
        is_seg = bool(seg_part)

        if is_seg:                           # 세그먼트 노드
            seg_n   = int(seg_part[0])
            fm_idx  = seg_mid_idx[base_id][seg_n]
            fm_pt   = lanelet[base_id]['waypoints'][fm_idx]
            fm_tail = lanelet[base_id]['waypoints'][fm_idx:]
        else:                                # 일반 노드
            fm_pt   = mid_pt[base_id]
            fm_tail = back_tail[base_id]

        # ── Node 마커
        markers.markers.append(
            Node(node_id, n, _as_list(fm_pt), (1, 1, 1, 1))
        )

        # ── Edge 마커
        for m, tgt_id in enumerate(edges):
            tgt_base, *tseg = tgt_id.split('_')
            tgt_seg = bool(tseg)

            if tgt_seg:                      # 세그 ↔ 세그 (두 점만)
                tsn  = int(tseg[0])
                tm_idx = seg_mid_idx[tgt_base][tsn]
                tm_pt  = lanelet[tgt_base]['waypoints'][tm_idx]
                pts = [_as_list(fm_pt), _as_list(tm_pt)]
            else:                            # 세그/일반 ↔ 일반
                tm_idx = mid_idx[tgt_base]
                front  = seg_front[tgt_base].get(0) or \
                         lanelet[tgt_base]['waypoints'][:tm_idx]
                concat = np.concatenate((fm_tail, front), axis=0)  # view 결합
                pts = concat.tolist()            # 한 번만 list 변환

            mk1, mk2 = Edge(n*100000 + m, pts, (0, 1, 0, 0.5))
            markers.markers.extend((mk1, mk2))

    return markers

def MicroLaneletGraphViz2(lanelet, graph): # original one
    array = MarkerArray()

    for n, (node_id, data) in enumerate(graph.items()):
        split = node_id.split('_')

        if len(split) == 1:
            id_ = split[0]
            from_idx = lanelet[id_]['idx_num'] // 2
            from_pts = lanelet[id_]['waypoints']
            marker = Node(node_id, n, from_pts[from_idx], (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

            for m, target_node_id in enumerate(data.keys()):
                split = target_node_id.split('_')
                pts = []

                if len(split) == 1:
                    target_id = split[0]
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = lanelet[target_id]['idx_num'] // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])
                else:
                    target_id = split[0]
                    cut_n = int(split[1])
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = sum(lanelet[target_id]['cut_idx'][cut_n]) // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])

                marker1, marker2 = Edge(n*100000+m, pts, (0.0, 1.0, 0.0, 0.5))
                array.markers.append(marker1)
                array.markers.append(marker2)

        else:
            id_ = split[0]
            cut_n = int(split[1])
            from_idx = sum(lanelet[id_]['cut_idx'][cut_n]) // 2
            from_pts = lanelet[id_]['waypoints']
            marker = Node(node_id, n, from_pts[from_idx], (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

            for m, target_node_id in enumerate(data.keys()):
                split = target_node_id.split('_')
                pts = []

                if len(split) == 1:
                    target_id = split[0]
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = lanelet[target_id]['idx_num'] // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])
                else:
                    target_id = split[0]
                    cut_n = int(split[1])
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = sum(lanelet[target_id]['cut_idx'][cut_n]) // 2
                    pts = [from_pts[from_idx], to_pts[to_idx]]

                marker1, marker2 = Edge(n*100000+m, pts, (0.0, 1.0, 0.0, 0.5))
                array.markers.append(marker1)
                array.markers.append(marker2)

    return array


def ConstructionSiteViz(construction_sites):
    array = MarkerArray()
    for n, points in enumerate(construction_sites):
        marker = Line('construction_site', n, 0.5, (1.0, 0.0, 0.0, 1.0))
        for pt in points:
            marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))
        array.markers.append(marker)

    return array


def Edge(n, points, color):
    if len(points) == 2:
        wx, wy = zip(*points)
        itp = QuadraticSplineInterpolate(list(wx), list(wy))
        pts = []
        for ds in np.arange(0.0, itp.s[-1], 0.5):
            pts.append(itp.calc_position(ds))
        points = pts

    marker1 = Line('edge_line', n, 0.5, color)
    for pt in points:
        marker1.points.append(Point(x=pt[0], y=pt[1], z=0.0))

    marker2 = Arrow('edge_arrow', n, (1.0, 2.0, 4.0), color)
    num = len(points)
    if num > 2:
        marker2.points.append(
            Point(x=points[-min(max(num, 3), 5)][0], y=points[-min(max(num, 3), 5)][1]))
    else:
        marker2.points.append(Point(x=points[-2][0], y=points[-2][1]))
    marker2.points.append(Point(x=points[-1][0], y=points[-1][1]))
    return marker1, marker2


def Node(id_, n, pt, color):
    marker = Text('graph_id', n, 2.5, color, id_)
    marker.pose.position = Point(x=pt[0], y=pt[1], z=1.0)
    return marker


def ID(id_, pt, color):
    marker = Text('id', int(id_), 2.5, color, id_)
    marker.pose.position = Point(x=pt[0], y=pt[1], z=1.0)
    return marker


def Waypoints(id_, points, color):
    marker = Points('waypoints', int(id_), 0.15, color)

    for pt in points:
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))
    return marker


def RefPath(waypoints, id_, color):
    marker = Points('ref_path', int(id_), 0.5, color)
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))
    return marker


def LocalPath(waypoints, id_, z, scale, color):
    marker = Line('local_path', int(id_), scale, color)
    marker.lifetime = rospy.Duration(2.0)
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=z))
    return marker


def Bound(ns, id_, n, points, type_, color):
    if type_ == 'solid':
        marker = Line('%s_%s' % (ns, id_), n, 0.15, color)
        for pt in points:
            marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))

    elif type_ == 'dotted':
        marker = Points('%s_%s' % (ns, id_), n, 0.15, color)
        for pt in points:
            marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))

    return marker


def Sphere(ns, id_, data, scale, color):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.position.x = data[0]
    marker.pose.position.y = data[1]
    marker.pose.position.z = 1.0
    marker.pose.orientation.w = 1.0 
    return marker


def Cube(ns, id_, scale, quaternion, color):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale*1.5
    marker.scale.y = scale*1.5
    marker.scale.z = scale*1.5
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.w = 1.0
    return marker


def CubeV(ns, id_, scale, quaternion, color):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale*1.3
    marker.scale.y = scale*1.3
    marker.scale.z = scale*0.9
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker


def Points(ns, id_, scale, color):
    marker = Marker()
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker


def Line(ns, id_, scale, color):
    marker = Marker()
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker


def Text(ns, id_, scale, color, text):
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.text = text
    marker.scale.z = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker


def Arrow(ns, id_, scale, color):
    marker = Marker()
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker


def EgoCarViz():
    marker = Marker()
    marker.header.frame_id = 'ego_car'
    marker.ns = 'car'
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = 'file://{}/obj/car.dae'.format(dir_path)
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = 2.0
    marker.scale.y = 2.0
    marker.scale.z = 2.0
    marker.color.r = 0.7
    marker.color.g = 0.7
    marker.color.b = 0.7
    marker.color.a = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 1.0
    quaternion = tf.transformations.quaternion_from_euler(
        0, 0, math.radians(90))
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    return marker

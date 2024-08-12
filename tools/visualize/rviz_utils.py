import os
import tf
import math
import numpy as np
import time

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

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
        marker = Sphere('obstacle', n, (round(pt[0],1), round(pt[1],1)), 2.1, color)
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


def TrafficLightViz(pt):
    data = (pt[0], pt[1])
    marker = Sphere('traffic_light', 0, data, 2.0, (1.0, 0.0, 0.0, 1.0))
    return marker

def FinalPathViz(waypoints):
    return FinalPath(waypoints, 999, 0.1, 0.2, (0.0, 1.0, 0.0, 1.0))

def LocalPathViz(waypoints):
    return FinalPath(waypoints, 999, 0.2, 0.4, (1.0, 0.0, 0.0, 1.0))

def ForwardPathViz(waypoints):
    return FinalPath(waypoints, 999, 0.2, 0.4, (1.0, 0.0, 1.0, 1.0))

def FinalPath(waypoints, id_, z, scale, color):
    marker = Line('final_path', int(id_), scale, color)
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=z))
    return marker

def CrosswalkViz(waypoints):
    marker = Line('crosswalk', 999, 0.4, (1.0, 0.2, 0.6, 1.0))
    for _, pt in enumerate(waypoints):
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.2))
    return marker

def StopLineViz(waypoints):
    marker = Line('stopline', 999, 0.4, (1.0, 1.0, 0.0, 1.0))
    for _, pt in enumerate(waypoints):
        marker.points.append(Point(x=pt[0], y=pt[1], z=0.2))
    return marker

def LocalPathSelectedViz(path):
    waypoints = zip(path.x, path.y)
    return LocalPath(waypoints, 999, 0.1, 0.2, (0.0, 1.0, 0.0, 1.0))


def LocalPathsViz(local_paths):
    array = MarkerArray()

    for n, path in enumerate(local_paths):
        waypoints = zip(path.x, path.y)
        marker = LocalPath(waypoints, n, 0.0, 0.1, (0.5, 0.5, 0.5, 1.0))
        array.markers.append(marker)

    return array


def RefPathViz(waypoints):
    return RefPath(waypoints, 999, (0.0, 0.0, 1.0, 1.0))


def RefPathsViz(lanelet, ref_paths, waypoints):
    array = MarkerArray()

    for n, (ref_n, ref_path) in enumerate(ref_paths.items()):
        waypoints_ = waypoints[ref_n]
        # color = (random.randint(0,255)/255.0, random.randint(0,255)/255.0, random.randint(0,255)/255.0, 1.0)
        color = (1.0, 1.0, 0.0, 1.0)
        if len(waypoints_) != 0:
            marker = RefPath(waypoints_, n, color)
            array.markers.append(marker)

    return array


def GoalViz(pt):
    marker = Text('goal', 0, 2.0, (1.0, 1.0, 1.0, 1.0), 'GOAL')
    marker.pose.position = Point(x=pt[0], y=pt[1], z=1.0)
    return marker


def ShortestPathViz(lanelet, shortest_path):
    array = MarkerArray()

    # pre_pt = None

    # for n, id_ in enumerate(shortest_path):
    #     split = id_.split('_')

    #     if len(split) == 1:
    #         t_id = split[0]
    #         idx = lanelet[t_id]['idx_num'] // 2
    #     else:
    #         t_id = split[0]
    #         cut_n = int(split[1])
    #         idx = sum(lanelet[t_id]['cut_idx'][cut_n]) // 2

    #     pt = lanelet[t_id]['waypoints'][idx]
    #     if pre_pt is not None:
    #         marker = Edge(90000000+n, [pre_pt, pt], (0.0, 1.0, 0.0))
    #         array.markers.append(marker)

    #     pre_pt = pt

    return array


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


def VectorMapVis(map_data):
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
        marker = Sphere('traifficlight_%s' %
                        (id_), 0, data, 0.1, (1.0, 1.0, 1.0, 0.5))
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
        marker = PostPoint('postpoint_%s' % (id_), 0, data,
                           0.2, 4.0, (1.0, 1.0, 1.0, 0.5))
        array.markers.append(marker)

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


def MicroLaneletGraphViz(lanelet, graph):
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


def FinalPath(waypoints, id_, z, scale, color):
    marker = Line('final_path', int(id_), scale, color)
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

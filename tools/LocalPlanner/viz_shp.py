from libs import cubic_spline_planner
from shapely.geometry import LineString
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

BL = [288350, 4143900]


def interpolate_points(geom, distance):  # Input gpd linestring
    if geom.geom_type == 'LineString':
        # print(geom.length)
        num_vert = int(round(geom.length / distance))
        if num_vert == 0:
            num_vert = 1
        return LineString(
            [geom.interpolate(float(n) / num_vert, normalized=True)
             for n in range(num_vert + 1)])


def get_coord_tuple(road):
    shpfile = gpd.read_file('./GIS/{}.shp'.format(road))  # read shp
    shpfile = shpfile.to_crs('epsg:32652')
    X, Y = [], []
    for ind in range(len(shpfile.geometry)):
        new = interpolate_points(shpfile.geometry[ind], 1)
        for x, y in new.coords:
            X.append(x)
            Y.append(y)

    return [*zip(X, Y)]


def get_coord():
    shpfile = gpd.read_file('./GIS/F.shp')  # read shp
    shpfile = shpfile.to_crs('epsg:32652')
    X, Y = [], []
    for ind in len(shpfile.geometry):
        new = interpolate_points(shpfile.geometry[ind], 5)
        for x, y in new.coords:
            X.append(x)
            Y.append(y)
    return X, Y


def Points(ns, id_, scale):
    marker = Marker()
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1
    return marker


def ros_viz():
    rospy.init_node('planner', anonymous=False)
    pub_lanes = rospy.Publisher('/PG_Lanes', Marker, queue_size=1, latch=True)

    marker = Points('PG', 1, 0.15)
    
    for x, y in get_coord_tuple('F'):
        marker.points.append(Point(x - BL[0], y - BL[1], 0.0))

    for x, y in get_coord_tuple('B'):
        marker.points.append(Point(x - BL[0], y - BL[1], 0.0))
    pub_lanes.publish(marker)

    rospy.spin()


if __name__ == '__main__':
    ros_viz()

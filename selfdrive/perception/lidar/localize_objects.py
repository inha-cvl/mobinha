#!/usr/bin/python
import rospy
import math
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

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
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    return marker

class LidarObject():
    def __init__(self):
        rospy.init_node('localize_lidar', anonymous=False)
        self.pub_object = rospy.Publisher('/lidar_ob', Marker, queue_size=1)
        rospy.Subscriber('/odom', Float32MultiArray, self.get_odom)
        rospy.Subscriber('/lidar/lidar/track_pillar_box', BoundingBoxArray, self.localize_object)
        self.pose = [0, 0, 0, 0, 0]
        rospy.spin()

    def get_odom(self, msg):
        self.pose = msg.data

    def localize_object(self, msg):
        marker = Points('OB', 1, 1.5)
        pose = self.pose
        for obj in msg.boxes:
            ob = obj.pose.position
            theta = math.radians(pose[2])
            x = ob.x * math.cos(theta) - ob.y * math.sin(theta) + pose[0]
            y = ob.x * math.sin(theta) + ob.y * math.cos(theta) + pose[1]
            marker.points.append(Point(x, y, 0))
        self.pub_object.publish(marker)


if __name__ == '__main__':
    LidarObject()


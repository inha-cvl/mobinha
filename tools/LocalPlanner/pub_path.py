import pickle
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

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

if __name__ == '__main__':
    rospy.init_node('path')
    pub_path = rospy.Publisher('/final_path', Marker, queue_size=1, latch=True)
    marker = Line('final_path', 1, 0.1, [0, 1, 0, 1])

    final_path = []

    with open('path.pkl', 'rb') as f:
        final_path = pickle.load(f)

    for pt in final_path:
        marker.points.append(Point(pt[0], pt[1], 0))

    print('pub')
    pub_path.publish(marker)

    rospy.spin()

    

#!/usr/bin/env python
  
import rospy
from sensor_msgs.msg import PointCloud2,PointCloud
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32

class lidarPaser:
    def __init__(self):
        rospy.init_node('lidar', anonymous=True)
    
        rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.pc1_pub = rospy.Publisher('/pc1',PointCloud, queue_size=1)
        
        
        rospy.spin()

    def callback(self, data):
        pc1_msg=PointCloud()
        for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
            tmp_point=Point32()
            tmp_point.x=p[0]
            tmp_point.y=p[1]
            tmp_point.z=p[2]
            pc1_msg.points.append(tmp_point)


        pc1_msg.header.frame_id=data.header.frame_id
        pc1_msg.header.stamp=data.header.stamp
        self.pc1_pub.publish(pc1_msg)
        print("point num : {}".format(len(pc1_msg.points)))
        
        

if __name__ == '__main__':
    try:
        lidar = lidarPaser()
    except rospy.ROSInterruptException:
        pass
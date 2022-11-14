import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from jsk_recognition_msgs.msg import BoundingBoxArray

class Convert():
    def __init__(self):
        rospy.init_node('lidar_tf', anonymous=True)
        get_point = rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.callback)
        # get_array = rospy.Subscriber('/pillar_box_array', BoundingBoxArray, self.marker_callback)
        get_array = rospy.Subscriber('/pillar_marker_array', MarkerArray, self.marker_callback)
        self.set_point = rospy.Publisher('/new_cloud', PointCloud2,queue_size=10)
        # self.set_new_point = rospy.Publisher('/new_points', BoundingBoxArray,queue_size=1)
        self.set_new_point = rospy.Publisher('/new_points', MarkerArray,queue_size=1)

        rospy.spin()

    def callback(self, data):
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'ego_car'
        self.set_point.publish(data)

    def marker_callback(self, data):
        for i in range(len(data.markers)):
            data.markers[i].header.stamp = rospy.Time.now()
        # for i in range(len(data.boxes)):
            # data.boxes[i].header.stamp = rospy.Time.now()
        # data.header.stamp = rospy.Time.now()
        # # data.header.frame_id = 'ego_car'
        self.set_new_point.publish(data)

if __name__ == '__main__':
    Convert()
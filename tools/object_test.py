import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox


class PublishBBox:
    def __init__(self):
        rospy.init_node('PublishBBox', anonymous=False)
        self.pub_bbox_array = rospy.Publisher(
            '/lidar/cluster_box', BoundingBoxArray, queue_size=1)
        self.bbox_arr = BoundingBoxArray()

    def publish(self):
        self.bbox_arr.header.frame_id = '1'
        bbox = BoundingBox()
        bbox.pose.position.x = 59.52236
        bbox.pose.position.y = -115.60573
        self.bbox_arr.boxes.append(bbox)
        self.pub_bbox_array.publish(self.bbox_arr)


if __name__ == '__main__':
    pb = PublishBBox()
    while not rospy.is_shutdown():
        pb.publish()
        (rospy.Rate(10)).sleep()
    rospy.spin()

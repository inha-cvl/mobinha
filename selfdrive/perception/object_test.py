import time
import sys
import rospy
import signal
from std_msgs.msg import String

from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from geometry_msgs.msg import PoseStamped

from selfdrive.message.messaging import *


class PublishBBox:
    def __init__(self):
        self.state = 'WAITING'
        rospy.Subscriber(
            '/mobinha/visualize/system_state', String, self.state_cb)
        sub_goal = rospy.Subscriber(
            '/move_base_simple/single_goal', PoseStamped, self.goal_cb)
        self.pub_bbox_array = rospy.Publisher(
            '/lidar/track_box', BoundingBoxArray, queue_size=1)
        self.x = 90
        self.y = 0
        self.get_goal = False

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALIZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)

    def goal_cb(self, msg):
        self.goal_pt = [msg.pose.position.x, msg.pose.position.y]
        self.get_goal = True

    def publish(self, CP):
        sm = StateMaster(CP)
        while not rospy.is_shutdown():
            sm.update()
            if self.state == 'START' and self.get_goal:
                self.x -= sm.CS.vEgo*0.05
                bbox_arr = BoundingBoxArray()
                bbox_arr.header.frame_id = 'world'
                bbox = BoundingBox()
                bbox.pose.position.x = self.x
                bbox.pose.position.y = self.y
                bbox_arr.boxes.append(bbox)
                self.pub_bbox_array.publish(bbox_arr)
            elif self.state == 'OVER':
                return 1
            time.sleep(0.1)


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('PublishBBox', anonymous=False)
    pb = PublishBBox()
    print("[{}] Created".format(pb.__class__.__name__))

    try:
        car_class = getattr(sys.modules[__name__], car)
        if pb.publish(car_class.CP) == 1:
            print("[{}] Over".format(pb.__class__.__name__))
            time.sleep(4)
            sys.exit(0)
    except Exception as e:
        print("[{} Error]".format(pb.__class__.__name__), e)
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(pb.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)

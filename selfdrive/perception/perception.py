import sys
import signal
import time
import rospy
from std_msgs.msg import String

from obstacle_detector import ObstacleDetector
from selfdrive.message.messaging import *


class Perception:
    def __init__(self):
        self.state = 'WAITING'
        sub_state = rospy.Subscriber('/state', String, self.state_cb)

    def perception(self):
        car = rospy.get_param('car_name', 'None')
        CP = getattr(sys.modules[__name__], car).CP
        sm = StateMaster(CP)
        obstacle_detector = ObstacleDetector(CP)

        while True:
            sm.update()
            if self.state == 'START':
                obstacle_detector.run(sm.CS)
                time.sleep(0.1)
            elif self.state == 'FINISH':
                return 1
            else:
                time.sleep(0.1)
                continue

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)


def signal_handler(sig, frame):
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Perception', anonymous=False)
    p = Perception()
    print("[{}] Created".format(p.__class__.__name__))

    try:
        if p.perception() == 1:
            print("[{}] Over".format(p.__class__.__name__))
            time.sleep(3)
            sys.exit(0)
    except Exception as e:
        print("[{} Error]".format(p.__class__.__name__), e)
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(p.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    main()

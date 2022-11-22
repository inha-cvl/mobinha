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

    def perception(self, CP):
        sm = StateMaster(CP)
        obstacle_detector = ObstacleDetector()

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
                print("[Perception Process] Start")
            elif str(msg.data) == 'INITIALZE':
                print("[Perception Process] Initialize")
        self.state = str(msg.data)


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Perception', anonymous=False)
    print("[Perception Process] Start")
    p = Perception()

    try:
        car_class = getattr(sys.modules[__name__], car)
        if p.perception(car_class.CP) == 1:
            print("Perception Process] Over")
            time.sleep(3)
            sys.exit(0)
    except Exception as e:
        print("[Perception ERROR] ", e)
    except KeyboardInterrupt:
        print("[Perception Process] Force Quit")
        sys.exit(0)


if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)

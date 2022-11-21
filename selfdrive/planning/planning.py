import sys
import signal
import time

import rospy
from std_msgs.msg import String
from path_planner import PathPlanner
from longitudinal_planner import LongitudinalPlanner

from selfdrive.message.messaging import *


def signal_handler(sig, frame):
    sys.exit(0)


class Planning:
    def __init__(self):
        self.state = 'WAITING'
        sub_state = rospy.Subscriber('/state', String, self.state_cb)

    def planning(self, CP):
        sm = StateMaster(CP)
        path_planner = PathPlanner(CP)
        longitudinal_planner = LongitudinalPlanner(CP)

        while True:
            if self.state == 'START':
                sm.update()
                pp = path_planner.run(sm)
                lgp = longitudinal_planner.run(sm, pp)
                time.sleep(0.1)  # 10Hz

                if pp == 2 and lgp == 2:
                    time.sleep(1)
                    print("[Planning Process] For Restart, please initialize")
                    if self.state == 'INITIALIZE':
                        # Re-Initialization of Path Planning self.State
                        path_planner.state = 'WAITING'
                        path_planner.get_goal = False
                        longitudinal_planner.local_path = None
            elif self.state == 'FINISH':
                return 1
            else:
                time.sleep(0.1)
                continue
            # rospy.spin()

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[Planning Process] Start")
            elif str(msg.data) == 'INITIALZE':
                print("[Planning Process] Initialize")
        self.state = str(msg.data)


def main(car):
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    print("[Planning Process] Created")
    rospy.init_node('Planning', anonymous=False)
    p = Planning()

    try:
        car_class = getattr(sys.modules[__name__], car)
        if p.planning(car_class.CP) == 1:
            print("[Planning Process] Over")
            time.sleep(3)
            sys.exit(0)
    except Exception as e:
        print("[Planning ERROR] ", e)
    except KeyboardInterrupt:
        print("[Planning Process] Force Quit")
        sys.exit(0)


if __name__ == "__main__":
    car = "NIRO"
    main(car)

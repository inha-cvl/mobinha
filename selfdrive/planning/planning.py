import sys
import signal
import time

import rospy
from std_msgs.msg import String, Int16MultiArray
from path_planner import PathPlanner
from longitudinal_planner import LongitudinalPlanner

from selfdrive.message.messaging import *


def signal_handler(sig, frame):
    sys.exit(0)


class Planning:
    def __init__(self):
        self.state = 'WAITING'
        self.planning_state = 'GOOD'
        sub_state = rospy.Subscriber('/state', String, self.state_cb)
        self.pub_planning_State = rospy.Publisher(
            '/planning_state', Int16MultiArray, queue_size=1)

    def planning(self):
        car = rospy.get_param('car_name', 'None')
        CP = getattr(sys.modules[__name__], car).CP
        sm = StateMaster(CP)
        path_planner = PathPlanner(CP)
        longitudinal_planner = LongitudinalPlanner(CP)

        pp = 0
        lgp = 0
        while True:
            if self.planning_state != 'GOOD':
                return 1
            sm.update()
            if self.state == 'START':
                pp = path_planner.run(sm)
                lgp = longitudinal_planner.run(sm, pp)
                time.sleep(0.1)  # 10Hz

                if pp == 2 and lgp == 2:
                    time.sleep(1)
                    print("[{}] For Restart, please initialize".format(
                        self.__class__.__name__))

            elif self.state == 'INITIALIZE':
                # Re-Initialization of Path Planning self.State
                path_planner.state = 'WAITING'
                path_planner.get_goal = False
                longitudinal_planner.local_path = None
                pp = 0
                lgp = 0

            elif self.state == 'TOR':
                time.sleep(1)
                print("[{}] Take Over Request, please initialize".format(
                    self.__class__.__name__))

            elif self.state == 'FINISH':
                return 1
            else:
                time.sleep(0.1)

            array = Int16MultiArray()
            array.data = [pp, lgp]
            self.pub_planning_State.publish(array)

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Planning', anonymous=False)
    p = Planning()
    print("[{}] Created".format(p.__class__.__name__))

    try:
        if p.planning() == 1:
            print("[{}] Over".format(p.__class__.__name__))
            time.sleep(3)
            sys.exit(0)
    except Exception as e:
        p.planning_state = 'BAD'
        print("[{} Error]".format(p.__class__.__name__), e)
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(p.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    main()

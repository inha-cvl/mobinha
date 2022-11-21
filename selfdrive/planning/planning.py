import sys
import signal
import time

from path_planner import PathPlanner
from longitudinal_planner import LongitudinalPlanner

from selfdrive.message.messaging import *


def planning(CP):
    sm = StateMaster(CP)
    path_planner = PathPlanner(CP)
    longitudinal_planner = LongitudinalPlanner(CP)

    while True:
        sm.update()
        pp = path_planner.run(sm)
        lgp = longitudinal_planner.run(sm, pp)
        time.sleep(0.1)  # 10Hz

        if pp == 2 and lgp == 2:
            time.sleep(1)
            user = input("[Planning Process] For Restart, Enter 1 Else 0")
            if user == 1 :
                #Re-Initialization of Path Planning State
                path_planner.state = 'WAITING'
                path_planner.get_goal = False
                longitudinal_planner.local_path = None
            else:
                return 1


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    print("[Planning Process] Start")
    rospy.init_node('Planning', anonymous=False)

    try:
        car_class = getattr(sys.modules[__name__], car)
        if planning(car_class.CP) == 1:
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

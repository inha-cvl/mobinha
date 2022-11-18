import sys
import signal
import time

from path_planner import PathPlanner
from longitudinal_planner import LongitudinalPlanner

from selfdrive.message.messaging import *


def planning(CP):
    sm = StateMaster(CP)
    dm = DrivingMaster()
    path_planner = PathPlanner(CP)
    longitudinal_planner = LongitudinalPlanner(CP)

    while True:
        sm.update()
        dm.update(sm)
        path_planner.run(sm, dm)
        longitudinal_planner.run(sm, dm)
        time.sleep(0.1)  # 10Hz

        if dm.CD.pathPlanningState == 2 and dm.CD.longitudinalPlanningState == 2:
            return 1


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Planning', anonymous=False)

    try:
        car_class = getattr(sys.modules[__name__], car)
        if planning(car_class.CP) == 1:
            sys.exit(0)
    except Exception as e:
        print("[Planning ERROR] ", e)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    car = "NIRO"
    main(car)

import sys
import signal

from path_planner import PathPlanner
# from longitudinal_planner import LongitudinalPlanner

from car.params import *
from car.states import *


def planning(CP):
    sm = StateMaster(CP)
    # path_planner = PathPlanner(CP)
    # longitudinal_planner = LongitudinalPlanner(CP)

    while True:
        sm.update()
        # path_planner.update()
        # longitudinal_planner.update()


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)

    try:
        car_class = getattr(sys.modules[__name__], car)
        CP = car_class.CP
        #CC = car_class.CC
    except Exception:
        print("[ERROR] could not load car values")
    except KeyboardInterrupt:
        sys.exit(0)

    planning(CP)


if __name__ == "__main__":
    car = "NIRO"
    main(car)

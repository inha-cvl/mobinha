import sys
import signal

from path_planner import PathPlanner
from longitudinal_planner import LongitudinalPlanner

from car.values import *
from message.messaging import *


class SignalInterruptHandler:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        sys.exit(0)


def planner(CP):
    sm = StateMaster()
    path_planner = PathPlanner(CP)
    longitudinal_planner = LongitudinalPlanner(CP)

    while True:
        sm.update()
        path_planner.update()
        longitudinal_planner.update()


def main(car):
    try:
        car_class = getattr(sys.modules[__name__], car)
        CP = car_class.CP
        #CC = car_class.CC
    except:
        print("[ERROR] could not load car values")

    planner(CP)


if __name__ == "__main__":
    car = "NIRO"
    main(car)

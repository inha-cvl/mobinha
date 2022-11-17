import sys
import signal
import time

from car.states import *
from car.params import *
from libs.localizer import Localizer
from libs.controller import Controller


def signal_handler(sig, frame):
    sys.exit(0)


def control(CP):
    sm = StateMaster()
    localizer = Localizer()
    controller = Controller(CP)

    while True:
        sm.update()
        localizer.run(sm.CS)
        controller.run(sm.CS)
        time.sleep(0.05)  # 20Hz


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    try:
        car_class = getattr(sys.modules[__name__], car)
        control(car_class.CP)
    except Exception:
        print("[ERROR] Could not load Car Params")
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    car = "NIRO"
    main(car)

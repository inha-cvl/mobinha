#!/usr/bin/python

import sys
import signal
import time

from selfdrive.message.messaging import *
from simulator_can import SimulatorCAN
from niro_can import NiroCAN



def car(car, CP):
    if car == "SIMULATOR":
        can = SimulatorCAN(CP)
    else:
        can = NiroCAN()

    while True:
        can.run()
        time.sleep(0.1)

def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    print("[Car Process] Start")

    try:
        car_class = getattr(sys.modules[__name__], car)
        car(car, car_class.CP)
    except Exception as e:
        print("[Car ERROR] ", e)
    except KeyboardInterrupt:
        print("[Car Process] Force Quit")
        sys.exit(0)


if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)

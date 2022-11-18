#!/usr/bin/python

import sys
import signal
import time

from selfdrive.message.messaging import *
from libs.simulate_ego import EgoSimulate


def simulator(CP):
    es = EgoSimulate(CP)

    while True:
        es.run()
        time.sleep(0.1)


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    try:
        car_class = getattr(sys.modules[__name__], car)
        simulator(car_class.CP)
    except Exception as e:
        print("[ERROR] ", e)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)

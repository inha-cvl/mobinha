import sys
import signal
import time
import rospy

from selfdrive.message.messaging import *
from libs.localizer import Localizer
from libs.controller import Controller


def control(CP):
    sm = StateMaster(CP)
    localizer = Localizer()
    controller = Controller(CP)

    while True:
        sm.update()
        localizer.run(sm)
        controller.run(sm)
        time.sleep(0.05)  # 20Hz


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Control', anonymous=False)

    try:
        car_class = getattr(sys.modules[__name__], car)
        control(car_class.CP)
    except Exception as e:
        print("[Control ERROR] ", e)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)

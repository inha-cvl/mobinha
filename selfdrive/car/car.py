#!/usr/bin/python

import sys
import signal
import time
import rospy
from std_msgs.msg import String

from selfdrive.message.messaging import *
from selfdrive.car.simulator_transceiver import SimulatorTransceiver
from selfdrive.car.niro_transceiver import NiroTransceiver


def signal_handler(sig, frame):
    sys.exit(0)


class Transceiver:
    def __init__(self):
        self.state = 'WAITING'
        sub_state = rospy.Subscriber('/state', String, self.state_cb)

    def transceiver(self, car, CP):
        if car == "SIMULATOR":
            can = SimulatorTransceiver(CP)
        else:
            can = NiroTransceiver

        while True:
            if self.state == 'START':
                can.run()
                time.sleep(0.1)
            elif self.state == 'FINISH':
                return 1
            else:
                time.sleep(0.1)
                continue

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[Car Process] Start")
            elif str(msg.data) == 'INITIALZE':
                print("[Car Process] Initialize")
        self.state = str(msg.data)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Car', anonymous=False)
    print("[Car Process] Created")
    t = Transceiver()

    try:
        car_class = getattr(sys.modules[__name__], car)
        if t.transceiver(car, car_class.CP) == 1:
            print("[CAR Process] Over")
            time.sleep(3)
            sys.exit(0)

    except Exception as e:
        print("[Car ERROR] ", e)
    except KeyboardInterrupt:
        print("[Car Process] Force Quit")
        sys.exit(0)


if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)

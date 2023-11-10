#!/usr/bin/python

import sys
import signal
import time
import rospy
import traceback
from std_msgs.msg import String

from selfdrive.message.messaging import *
from selfdrive.car.simulator_transceiver import SimulatorTransceiver
from selfdrive.car.morai_transceiver import MoraiTransceiver
from selfdrive.car.ioniq_transceiver import IoniqTransceiver


def signal_handler(sig, frame):
    sys.exit(0)


class Transceiver:
    def __init__(self):
        self.state = 'WAITING'
        self.need_init = True
        self.tick = {0: 0, 0.01: 0, 0.02: 0, 0.05: 0, 0.1: 0, 1: 0}
        rospy.Subscriber('/mobinha/visualize/system_state',
                         String, self.state_cb)

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False
        
    def transceiver(self):
        can = None
        timer = None
        cm = None
        while True:
            if self.state == 'INITIALIZE':
                if self.need_init:
                    cm, can, timer = self.init()
            elif self.state == 'START':
                if self.timer(timer):
                    self.need_init = True
                    cm.update()
                    can.run(cm)
            elif self.state == 'OVER':
                return 1
            # time.sleep(timer)

    def init(self):
        self.need_init = False
        car = rospy.get_param('car_name', None)
        map = rospy.get_param('map_name', None)
        CP = (getattr(sys.modules[__name__], car)(map)).CP
        cm = ControlMaster()
        timer = 0.1
        if car == "SIMULATOR":
            can = SimulatorTransceiver(CP)
        elif car == "MORAI":
            can = MoraiTransceiver()
        elif car == "IONIQ":
            can = IoniqTransceiver(CP)
            timer = 0
        return cm, can, timer

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALIZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Car', anonymous=False)
    t = Transceiver()
    print("[{}] Created".format(t.__class__.__name__))

    try:
        if t.transceiver() == 1:
            print("[Car Process] Over")
            time.sleep(4)
            sys.exit(0)

    except Exception:
        print("[{} Error]".format(t.__class__.__name__), traceback.print_exc())
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(t.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    main()

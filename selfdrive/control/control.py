import sys
import signal
import time
import rospy
from std_msgs.msg import String

from selfdrive.message.messaging import *
from selfdrive.control.localizer import Localizer
from selfdrive.control.controller import Controller


class Control:
    def __init__(self):
        self.state = 'WAITING'
        self.need_init = True
        rospy.Subscriber(
            '/mobinha/visualize/system_state', String, self.state_cb)

    def control(self):
        sm = None
        localizer = None
        controller = None
        while True:
            if self.state == 'INITIALIZE':
                if self.need_init:
                    sm, localizer, controller = self.init()
            elif self.state == 'START':
                self.need_init = True
                sm.update()
                localizer.run(sm)
                controller.run(sm)
                time.sleep(0.05)  # 20Hz
            elif self.state == 'OVER':
                return 1
            else:
                time.sleep(0.1)
                continue

    def init(self):
        self.need_init = False
        car = rospy.get_param('car_name', 'None')
        map = rospy.get_param('map_name', 'None')
        CP = (getattr(sys.modules[__name__], car)(map)).CP
        sm = StateMaster(CP)
        localizer = Localizer()
        controller = Controller(CP)

        return sm, localizer, controller

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALIZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)


def signal_handler(sig, frame):
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Control', anonymous=False)
    c = Control()
    print("[{}] Created".format(c.__class__.__name__))

    try:
        if c.control() == 1:
            print("[{}] Over".format(c.__class__.__name__))
            time.sleep(4)
            sys.exit(0)
    except Exception as e:
        print("[{} Error]".format(c.__class__.__name__), e)
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(c.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    main()

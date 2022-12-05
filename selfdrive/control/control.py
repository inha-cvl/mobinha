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
        sub_state = rospy.Subscriber('/state', String, self.state_cb)

    def control(self, CP):
        sm = StateMaster(CP)
        localizer = Localizer()
        controller = Controller(CP)

        while True:
            sm.update()
            if self.state == 'START':
                localizer.run(sm)
                controller.run(sm)
                time.sleep(0.05)  # 20Hz
            elif self.state == 'FINISH':
                return 1
            else:
                time.sleep(0.1)
                continue

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALZE':
                print("[{}] Initialize".format(self.__class__.__name__))
        self.state = str(msg.data)


def signal_handler(sig, frame):
    sys.exit(0)


def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Control', anonymous=False)
    c = Control()
    print("[{}] Created".format(c.__class__.__name__))

    try:
        car_class = getattr(sys.modules[__name__], car)
        if c.control(car_class.CP) == 1:
            print("[{}] Over".format(c.__class__.__name__))
            time.sleep(3)
            sys.exit(0)
    except Exception as e:
        print("[{} Error]".format(c.__class__.__name__), e)
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(c.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)
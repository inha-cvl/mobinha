import sys
import signal
import time
import rospy
import traceback
from std_msgs.msg import String, Int8

from obstacle_detector import ObstacleDetector
from selfdrive.message.messaging import *


class Perception:
    def __init__(self):
        self.state = 'WAITING'
        self.need_init = True
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0, 0.05: 0, 0.02: 0}
        rospy.Subscriber(
            '/mobinha/visualize/system_state', String, self.state_cb)
        self.pub_perception_State = rospy.Publisher(
            '/mobinha/perception_state', Int8, queue_size=10)

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False
        
    def perception(self):
        state = 0
        sm = None
        obstacle_detector = None
        while True:
            if self.state == 'INITIALIZE':
                state = 0
                if self.need_init:
                    sm, obstacle_detector = self.init()
            elif self.state == 'START':
                if self.timer(0.1):
                    self.need_init = True
                    sm.update()
                    obstacle_detector.run(sm.CS)
                    state = 1
            elif self.state == 'OVER':
                state = 0
                return 1
            else:
                state = 0
                time.sleep(0.1)
                #continue

            self.pub_perception_State.publish(Int8(state))

    def init(self):
        self.need_init = False
        car = rospy.get_param('car_name', 'None')
        map = rospy.get_param('map_name', 'None')
        CP = (getattr(sys.modules[__name__], car)(map)).CP
        sm = StateMaster(CP)
        obstacle_detector = ObstacleDetector(CP)
        return sm, obstacle_detector


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
    rospy.init_node('Perception', anonymous=False)
    p = Perception()
    print("[{}] Created".format(p.__class__.__name__))

    try:
        if p.perception() == 1:
            print("[{}] Over".format(p.__class__.__name__))
            time.sleep(4)
            sys.exit(0)
    except Exception:
        print("[{} Error]".format(p.__class__.__name__), traceback.print_exc())
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(p.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    main()

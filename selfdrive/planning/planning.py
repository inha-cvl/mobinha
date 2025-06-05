import sys
import signal
import time
import threading
import traceback
import rospy
from std_msgs.msg import String, Int16MultiArray
from path_planner import PathPlanner
from longitudinal_planner import LongitudinalPlanner

from selfdrive.message.messaging import *

from tools.morai.gen_ros.scripts import mobinha_planner

def signal_handler(sig, frame):
    sys.exit(0)

class Planning:
    def __init__(self):
        self.state = 'WAITING'
        self.planning_state = 'GOOD'
        self.morai_use = False
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0, 0.05: 0, 0.02: 0}
        rospy.Subscriber(
            '/mobinha/visualize/system_state', String, self.state_cb)
        self.pub_planning_State = rospy.Publisher(
            '/mobinha/planning_state', Int16MultiArray, queue_size=1)

        self.sm = None
        self.path_planner = None
        self.longitudinal_planner = None

    def timer(self, sec):
        if time.time() - self.tick[sec] > sec:
            self.tick[sec] = time.time()
            return True
        else:
            return False
    
    def planning(self):
        pp = 0
        lgp = 0
        
        while True:
            if self.planning_state != 'GOOD':
                return 1
            if self.state == 'INITIALIZE':
                self.init()
                pp = 0
                lgp = 0
            elif self.state == 'START':
                if self.timer(0.1):
                    self.sm.update()
                    pp, my_lane_id, global_ids, local_path, lmap, tmap = self.path_planner.run(self.sm)
                    lgp = self.longitudinal_planner.run(self.sm, lmap, tmap, my_lane_id, global_ids, pp, local_path) #sm, lmap, pp=0, local_path=None

                    array = Int16MultiArray()
                    array.data = [pp, lgp]
                    self.pub_planning_State.publish(array)

                if pp == 2 and lgp == 2:
                    time.sleep(1)
                    #print("[{}] For Restart, please initialize".format(self.__class__.__name__))
            
            elif self.state == 'RESET':
                self.path_planner.state = 'READY'
                self.path_planner.get_goal = False
                self.path_planner.global_path = None
                self.path_planner.local_path = None
                array = Int16MultiArray()
                array.data = [3,0]
                self.pub_planning_State.publish(array)

            elif self.state == 'TOR':
                time.sleep(1)
                print("[{}] Take Over Request, please initialize".format(
                    self.__class__.__name__))

            elif self.state == 'OVER':
                return 1
            else:
                time.sleep(0.1)


    def init(self):
        car = rospy.get_param('car_name', 'None')
        map = rospy.get_param('map_name', 'None')
        CP = (getattr(sys.modules[__name__], car)(map)).CP
        if not self.morai_use and car == 'MORAI':
            self.morai_use = True
            cm = ControlMaster()
            t = threading.Thread(target=mobinha_planner.run, args=(cm,))
            t.start()
        self.sm = StateMaster(CP)
        self.path_planner = PathPlanner(CP)
        self.longitudinal_planner = LongitudinalPlanner(CP)

    def state_cb(self, msg):
        if self.state != str(msg.data):
            if str(msg.data) == 'START':
                print("[{}] Start".format(self.__class__.__name__))
            elif str(msg.data) == 'INITIALIZE':
                print("[{}] Initialize".format(self.__class__.__name__))
            elif str(msg.data) == 'RESET':
                print("[{}] RESET".format(self.__class__.__name__))
        self.state = str(msg.data)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('Planning', anonymous=False)

    p = Planning()
    print("[{}] Created".format(p.__class__.__name__))

    try:
        if p.planning() == 1:
            print("[{}] Over".format(p.__class__.__name__))
            time.sleep(4)
            sys.exit(0)
    except Exception:
        p.planning_state = 'BAD'
        print("[{} Error]".format(p.__class__.__name__), traceback.print_exc())
    except KeyboardInterrupt:
        print("[{}] Force Quit".format(p.__class__.__name__))
        sys.exit(0)


if __name__ == "__main__":
    main()
import sys
import signal
import time

from obstacle_detector import ObstacleDetector

from selfdrive.message.messaging import *

def perception(CP):
    sm = StateMaster(CP)
    obstacle_detector = ObstacleDetector()

    while True:
        sm.update()
        obstacle_detector.run(sm)
        time.sleep(0.1)

def signal_handler(sig, frame):
    sys.exit(0)

def main(car):
    signal.signal(signal.SIGINT, signal_handler)
    print("[Perception Process] Start")
    rospy.init_node('Perception', anonymous=False)

    try:
        car_class = getattr(sys.modules[__name__], car)
        perception(car_class.CP)
    except Exception as e:
        print("[Perception ERROR] ", e)
    except KeyboardInterrupt:
        print("[Perception Process] Force Quit")
        sys.exit(0)

if __name__ == "__main__":
    car = "SIMULATOR"
    main(car)
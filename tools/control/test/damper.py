import rospy
import time
from std_msgs.msg import Float32


class Controller():
    def __init__(self):
        self.steer = {'state' : False , 'current' : 0}

        rospy.init_node('thread_test', anonymous=True)
        rospy.Subscriber('test', Float32, self.callback)
        rospy.spin()


    def callback(self, data):
        steer = self.steer
        data = data.data * 100
        step = 5
        if not steer['state']:
            steer['state'] = True
            if steer['current'] > data:
                step = -step
            for i in range(steer['current'], int(data), step):
                print(i/100)
                time.sleep(0.005)
            print(data/100)
            steer['state'] = False
            steer['current'] = int(data)
            self.steer = steer
        else:
            return

if __name__ == '__main__':
    a = Controller()

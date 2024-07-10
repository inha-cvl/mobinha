
import numpy as np
import time
import rospy
from std_msgs.msg import Float32


class Velocity_Planner:
    def __init__(self):
        self.pub = rospy.Publisher('/target_velocity', Float32, queue_size=1) 

        rospy.Subscriber('/current_velocity', Float32, self.current_velocity_cb)
        self.current_v = 0


        # ACC params
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        window_size = 1
        self.dis_history = [0]*window_size # for history & integral term

        self.dt = 0.01

        self.output = 0

    def current_velocity_cb(self, msg):
        self.current_v = msg.data


    def acc(self, car_pos, tar_pos):
        ## car_pos, tar_pos는 모두 np.array 형식
        dis = np.linalg.norm(tar_pos - car_pos)
        
        derivative = (dis - self.dis_history[-1]) / self.dt

        self.output = (self.Kp * dis) + (self.Ki * sum(self.dis_history)) + (self.Kd * derivative)

        # update
        self.dis_history[-1] = dis

        
    def target_velocity_publish(self):
        msg = Float32
        msg.data = self.output
        self.pub.publish(msg)
    
    

    

    

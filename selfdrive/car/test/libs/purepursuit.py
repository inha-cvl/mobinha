import rospy
from std_msgs.msg import Float32
from numpy.linalg import norm
from math import sin, cos, atan2, radians, degrees
import numpy as np

class PurePursuit:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, path):
        print("Pure Pursuit model initiated!")
        self.L = 3

    def run(self, vEgo, path, position, yawRate, cte):
       
        lfd = 5 + 1 * vEgo / 3.6  
        lfd = np.clip(lfd, 4, 60)  
        steering_angle = 0.  
        lx, ly = path[0] 
        
        for point in path:
            diff = np.asarray((point[0]-position[0], point[1]-position[1]))
            rotation_matrix = np.array(
                ((np.cos(-radians(yawRate)), -np.sin(-radians(yawRate))), (np.sin(-radians(yawRate)),  np.cos(-radians(yawRate)))))
            rotated_diff = rotation_matrix.dot(diff)
            if rotated_diff[0] > 0: 
                dis = np.linalg.norm(rotated_diff-np.array([0, 0]))
                if dis >= lfd: 
                    theta = np.arctan2(rotated_diff[1], rotated_diff[0]) 
                    steering_angle = np.arctan2(2*self.L*np.sin(theta), lfd)
                    # steering_angle = steering_angle + np.arctan2(0.1 * cte, vEgo) if vEgo > 6 else steering_angle
                    lx = point[0]  
                    ly = point[1]  
                    break
        return degrees(steering_angle), (lx, ly) 
    
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
        self.prev_angle = None

    def run(self, vEgo, path, position, yawRate, cte):
        if vEgo*3.6 < 12:
            lfd = 2.0
        else:
            lfd = 10 + 3 * vEgo**0.8 / 3.6

        print(f"CTE:{cte:.2f}")
        # print(f"       lfd:{lfd:.2f}")
        # print(f"                  vEgo:{vEgo*3.6:.2f}")
        lfd = np.clip(lfd, 4, 60)  
        steering_angle = 0.  
        lx, ly = path[0] 
        
        for point in path:
            diff = np.asarray((point[0]-position[0], point[1]-position[1]))
            rotation_matrix = np.array(
                ((np.cos(-radians(yawRate)), -np.sin(-radians(yawRate))), (np.sin(-radians(yawRate)),  np.cos(-radians(yawRate)))))
            rotated_diff = rotation_matrix.dot(diff)
            dis = np.linalg.norm(rotated_diff-np.array([0, 0]))
            if dis >= lfd: 
                theta = np.arctan2(rotated_diff[1], rotated_diff[0]) 
                steering_angle = np.arctan2(2*self.L*np.sin(theta), lfd)
                steering_angle = steering_angle + np.arctan2(0.1 * cte, vEgo) if vEgo > 6 else steering_angle
                lx = point[0]  
                ly = point[1]  
                break
        
        # # smoothing
        # alpha = 0.9
        # if self.prev_angle is None:
        #     self.prev_angle = steering_angle
        # else:
        #     steering_angle = alpha * steering_angle + (1 - alpha) * self.prev_angle
        #     self.prev_angle = steering_angle

        # factor = 1.3


        factor = 1
        val = np.clip(abs(cte)/2, -0.4, 0.4)
        if cte > 0:
            if steering_angle > 0:
                factor = 1 + val
            else:
                factor = 1 - val
        else:
            if steering_angle > 0:
                factor = 1 - val
            else:
                factor = 1 + val
        print("                                ", factor)

        return degrees(factor*steering_angle), (lx, ly) 
    
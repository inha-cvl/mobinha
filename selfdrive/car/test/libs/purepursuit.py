from math import sin, cos, atan2, radians, degrees
import numpy as np
import scipy.linalg as la
from scipy.signal import cont2discrete

class PurePursuit:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, path):
        print("Pure Pursuit model initiated!")
        self.L = 3
        self.prev_angle = None
        self.path = path
        

    def run(self, vEgo, path, idx, position, yawRate, cte):
        # lfd = 1.8 # for speed 8km, factor 1
        # lfd = 2 # for speed 10
        # lfd = 4 # for speed 20, factor 1.2
        lfd = 6 # for speed 24, factor 1.2  
        # lfd = 6 # for speed 30, factor 1.4
        # lfd = 7 # for speed 40, factor 1.3
        # lfd = 10 # for speed 50, factor 1.3


        # print(f"CTE:{cte:.2f}")
        lfd = np.clip(lfd, 4, 60)  
        steering_angle = 0.  
        lx, ly = path[0] 
        
        for point in path[idx:]:
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

        self.yaw_list.append(0)


        factor = 1.2

        return degrees(factor*steering_angle), (lx, ly)

    
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
        # lfd = 1.8 # for speed 8km, factor 1
        # lfd = 2 # for speed 10
        # lfd = 4 # for speed 20, factor 1.2
        lfd = 6 # for speed 24, factor 1.2  
        # lfd = 6 # for speed 30, factor 1.4

        # lfd = 7 # for speed 40, factor 1.3
        # lfd = 10 # for speed 50, factor 1.3


        print(f"CTE:{cte:.2f}")
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


        factor = 1.2

        return degrees(factor*steering_angle), (lx, ly) 
    

    def run_experimental(self, vEgo, path, idxEgo, posEgo, yawEgo, cte):
        # lfd = 1.8 # for speed 8km, factor 1
        # lfd = 2 # for speed 10
        # lfd = 4 # for speed 20, factor 1.2
        lfd = 6 # for speed 24, factor 1.2  
        # lfd = 6 # for speed 30, factor 1.4
        # lfd = 7 # for speed 40, factor 1.3
        # lfd = 10 # for speed 50, factor 1.3


        print(f"CTE:{cte:.2f}")
        heading = yawEgo
        x, y = posEgo

        resolution = 0.1
        lookahead_dis = 6 # [m]
        lookahead_idx = int(lookahead_dis/resolution)

        # Target index 결정
        target_idx = min(idxEgo + lookahead_idx, len(path)-1)
        if target_idx >= len(path)-1:
            target_idx = len(path)-1

        # 각도 계산
        target_x, target_y = path[target_idx]
        tmp = degrees(atan2(target_y - y, target_x - x)) % 360
        alpha = tmp - heading
        angle = atan2(2.0 * self.L * sin(radians(alpha)), lookahead_dis)
        steer = degrees(angle) 


        return steer, (target_x, target_y)
import rospy
from std_msgs.msg import Float32
from numpy.linalg import norm
from math import sin, cos, atan2, radians, degrees
import numpy as np
from libs.interpolate import interpolate


class PurePursuit:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, CP):
        # niro
        self.L = CP.wheelbase
        self.temp_lx = None
        self.temp_ly = None

        self.isLaneChange = False
        self.isBank = False
        rospy.Subscriber('/tmp_target_lfc', Float32, self.target_lfc_cb)
        rospy.Subscriber('/tmp_target_k', Float32, self.target_k_cb)
        rospy.Subscriber('/mobinha/avoid_gain', Float32, self.avoid_gain_cb)
        self.avoid_gain = 0.0

        self.k = CP.lateralTuning.lqr.k
        self.wheel_base = CP.wheelbase
        self.Lfc = CP.lateralTuning.lqr.l
        self.k_curva = 20.0

        self.cur_curvature = 0.0

        self.prev_angle = None

    def target_lfc_cb(self, msg):
        self.Lfc = msg.data

    def target_k_cb(self, msg):
        self.k = msg.data

    def avoid_gain_cb(self, msg):
        self.avoid_gain = msg.data

    def lane_change_cb(self, msg):
        if msg.data == 1:
            self.isLaneChange = True
        else:
            self.isLaneChange = False

    def bank_cb(self, msg):
        if msg.data == 1:
            self.isBank = True
        else:
            self.isBank = False

    def euc_distance(self, pt1, pt2):
        return norm([pt2[0] - pt1[0], pt2[1] - pt1[1]])

    def find_nearest_idx(self, pts, pt):
        min_dist = float('inf')
        min_idx = 0
        for idx, pt1 in enumerate(pts):
            dist = self.euc_distance(pt1, pt)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        return min_idx

    def get_local(self, my_x, my_y, yaw, path):
        in_x, in_y = path
        new_x = (in_x - my_x) * cos(-yaw) - (in_y - my_y)*sin(-yaw)
        new_y = (in_x - my_x) * sin(-yaw) + (in_y - my_y)*cos(-yaw)
        return (new_x, new_y)

    def run(self, vEgo, path, position, yawRate, cte):
        if vEgo*3.6 < 17.5:
            lfd = 3.6
        else:
            lfd = 6 + 2.3 * vEgo**0.8 / 3.6  
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
                steering_angle = steering_angle + np.arctan2(0.2 * cte, vEgo) if vEgo > 6 else steering_angle
                lx = point[0]  
                ly = point[1]  
                break
        
        # smoothing
        alpha = 0.9
        if self.prev_angle is None:
            self.prev_angle = steering_angle
        else:
            steering_angle = alpha * steering_angle + (1 - alpha) * self.prev_angle
            self.prev_angle = steering_angle

        factor = 1.3
        if vEgo*3.6 < 25:
            factor = 1.00
        return degrees(factor*steering_angle), (lx, ly) 
    
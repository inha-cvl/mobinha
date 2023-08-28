import rospy
from std_msgs.msg import Float32
from numpy.linalg import norm
from math import sin, cos, atan2, radians, degrees
import numpy as np
from libs.interpolate import interpolate


class HybridPPStanleyController:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, CP):
        self.L = CP.wheelbase

        self.k = CP.lateralTuning.lqr.k
        self.Lfc = CP.lateralTuning.lqr.l
        self.k_curva = 20.0

        self.cur_curvature = 0.0

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

    def stanley_run(self, vEgo, path, position, yawRate):
        lfd = self.Lfc+self.k*vEgo
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
                    steering_angle = np.arctan2(
                        2*self.L*np.sin(theta), lfd)
                    lx = point[0]
                    ly = point[1]
                    break
        return degrees(steering_angle), (lx, ly)
    
    def pp_run(self, vEgo, path, position, yawRate):
        lfd = self.Lfc+self.k*vEgo
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
                    steering_angle = np.arctan2(
                        2*self.L*np.sin(theta), lfd)
                    lx = point[0]
                    ly = point[1]
                    break
        return degrees(steering_angle), (lx, ly)
    
    def run(self, vEgo, path, position, yawRate):
        pp_wheel_angle, lah_pt = self.pp_run(
            vEgo, path, position, yawRate)
        stanley_wheel_angle = self.stanley_run(
            vEgo, path, position, yawRate)
        
        #R low is
        k_pp = .5
        k_stanley = .5
        
        hybrid_wheel_angle = k_pp*pp_wheel_angle + k_stanley*stanley_wheel_angle
        return hybrid_wheel_angle
        
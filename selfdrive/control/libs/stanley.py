import rospy
from std_msgs.msg import Float32
from numpy.linalg import norm
from math import sin, cos, atan2, radians, degrees
import numpy as np
from libs.interpolate import interpolate

class StanleyController:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, CP):
        self.k = CP.lateralTuning.stanley.k  # Stanley gain
        self.wheel_base = CP.wheelbase
        self.lqr_k = CP.lateralTuning.lqr.k
        self.Lfc = CP.lateralTuning.lqr.l

        # TODO: Add subscribers or other initializations if necessary

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

    def cross_track_error(self, closest_point, ahead_point, current_point):
        vector1 = np.array([ahead_point[0] - closest_point[0], ahead_point[1] - closest_point[1]])
        vector2 = np.array([current_point[0] - closest_point[0], current_point[1] - closest_point[1]])
        crosstrack_error = np.cross(vector1, vector2) / np.linalg.norm(vector1)
        return crosstrack_error

    def run(self, vEgo, path, position, yaw):
        lfd = self.Lfc+self.lqr_k*vEgo
        lfd = int(np.clip(lfd, 4, 60))
        closest_idx = self.find_nearest_idx(path, position)
        closest_point = path[closest_idx]
        
        # Choose a point ahead in the path as reference
        look_ahead_idx = min(len(path) - 1, closest_idx + 10)
        ahead_point = path[look_ahead_idx]

        # Calculate cross track error
        cte = self.cross_track_error(closest_point, ahead_point, position)
        # print("cte: ", cte)
        # Calculate the heading error
        theta_e = atan2(ahead_point[1] - closest_point[1], ahead_point[0] - closest_point[0])
        theta = radians(yaw)
        theta = theta % (2 * np.pi)
        if theta > np.pi:
            theta -= 2 * np.pi
        # print("theta_e: ", theta_e, "theta: ", theta)
        heading_error = theta_e - theta
        # print("heading_error: ", heading_error)

        # Ensure heading_error is within [-pi, pi]
        heading_error = atan2(sin(heading_error), cos(heading_error))

        # Stanley control law
        steering_angle = heading_error + atan2(-self.k * cte, vEgo)

        # Convert to degrees and return
        return degrees(steering_angle)

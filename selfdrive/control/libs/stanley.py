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
    def calculate_cte(self, pointA, pointB, pointP):
        Ax, Ay = pointA
        Bx, By = pointB
        Px, Py = pointP

        numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
        denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
        # return numerator / denominator if denominator != 0 else 0
        cte = numerator / denominator if denominator != 0 else 0
        # Calculate cross product to find the sign
        cross_product = (Bx - Ax) * (Py - Ay) - (By - Ay) * (Px - Ax)

        if cross_product > 0:
            return -cte  # Point P is on the left side of line AB
        elif cross_product < 0:
            return cte  # Point P is on the right side of line AB
        else:
            return 0  # Point P is on the line AB
    
    def get_front_wheel_position(self, rear_wheel_position, yaw):
        dx = self.wheel_base * cos(yaw)
        dy = self.wheel_base * sin(yaw)
        front_wheel_position = (rear_wheel_position[0] + dx, rear_wheel_position[1] + dy)
        return front_wheel_position

    def run(self, vEgo, path, position, yaw):
        front_wheel_position = self.get_front_wheel_position(position, radians(yaw))
        # lfd = self.Lfc+self.lqr_k*vEgo
        # lfd = int(np.clip(lfd, 4, 60))
        closest_idx = self.find_nearest_idx(path, front_wheel_position)
        closest_point = path[closest_idx]
        
        # Choose a point ahead in the path as reference
        # look_ahead_idx = min(len(path) - 1, closest_idx + lfd)
        ahead_point = path[closest_idx+2]

        # Calculate cross track error
        cte = self.calculate_cte(closest_point, ahead_point, front_wheel_position)
        # print("cte: ", round(cte,2))
        # Calculate the heading error
        theta_e = atan2(ahead_point[1] - closest_point[1], ahead_point[0] - closest_point[0])
        theta = radians(yaw)
        theta = theta % (2 * np.pi)
        if theta > np.pi:
            theta -= 2 * np.pi
        # print("theta_e: ", round(theta_e,4), "theta: ", round(theta,4))
        heading_error = theta_e - theta
        # print("heading_error: ", round(heading_error,4))

        # Ensure heading_error is within [-pi, pi]
        heading_error = atan2(sin(heading_error), cos(heading_error))

        # Stanley control law
        # k higher -> more aggressive for cte
        steering_angle = heading_error + atan2(self.k * cte, vEgo) if vEgo > 5 else heading_error
        if vEgo > 5:
            print("steering: ", round(degrees(heading_error)),"+",round(degrees(atan2(self.k * cte, vEgo))), "=", round(degrees(steering_angle)))
        else:
            print("steering: ", round(degrees(heading_error)))
        # print(front_wheel_position)
        # Convert to degrees and return
        return degrees(steering_angle)

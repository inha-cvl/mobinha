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

        self.k = CP.lateralTuning.lqr.k
        self.wheel_base = CP.wheelbase
        self.Lfc = CP.lateralTuning.lqr.l
        self.k_curva = 20.0

        self.cur_curvature = 0.0

    def target_lfc_cb(self, msg):
        self.Lfc = msg.data

    def target_k_cb(self, msg):
        self.k = msg.data

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

    def run(self, x, y, l_idx, yaw, v, path):
        yaw = radians(yaw)

        l_idx = self.find_nearest_idx(path, (x, y))

        step = 10
        if (len(path) - len(path[:l_idx])) > step:
            _, _, _, self.cur_curvature = interpolate(
                path[l_idx:l_idx+step], precision=0.5)
        else:  # end path
            self.cur_curvature = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        lx, ly = path[l_idx]

        v = max(v, 5.0 * self.KPH_TO_MPS)
        Lf = min(self.k * v + self.Lfc, 30.0)  # lookahead

        rear_x = x - ((self.L / 2) * cos(yaw))
        rear_y = y - ((self.L / 2) * sin(yaw))

        dist = 0
        while dist <= Lf and l_idx < len(path):
            lx = path[l_idx][0]
            ly = path[l_idx][1]
            dist = self.euc_distance((x, y), (lx, ly))
            l_idx += 1

        # Original Pure Pursuit
        alpha = atan2(ly - rear_y, lx - rear_x) - yaw
        pp_angle = atan2(2.0 * self.L * sin(alpha) / Lf, 1.0)

        curvature_control = 0.1*self.k_curva*self.cur_curvature[0] \
            + 0.15*self.k_curva*self.cur_curvature[1] \
            + 0.2*self.k_curva*self.cur_curvature[2] \
            + 0.35*self.k_curva*self.cur_curvature[3] \
            + 0.2*self.k_curva*self.cur_curvature[4]

        curvature_control = np.abs(curvature_control)
        if pp_angle >= 0:
            angle = degrees(pp_angle) + curvature_control
        elif pp_angle < 0:
            angle = degrees(pp_angle) - curvature_control

        # Set min/max
        angle = max(min(angle, 35.0), -35.0)

        return angle, (lx, ly)

    def run2(self, vEgo, path, position, yawRate):
        lfd = self.k*vEgo
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
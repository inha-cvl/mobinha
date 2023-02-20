import rospy
from std_msgs.msg import Float32
from numpy.linalg import norm
from math import sin, cos, atan2, radians, degrees
from numpy import abs
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

    def run(self, x, y, yaw, v, path):
        yaw = radians(yaw)

        i = self.find_nearest_idx(path, (x, y))

        step = 10
        if (len(path) - len(path[:i])) > step:
            _, _, _, self.cur_curvature = interpolate(
                path[i:i+step], precision=0.5)
        else:  # end path
            self.cur_curvature = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        lx, ly = path[i]

        v = max(v, 5.0 * self.KPH_TO_MPS)
        Lf = min(self.k * v + self.Lfc, 30.0)  # lookahead

        rear_x = x - ((self.L / 2) * cos(yaw))
        rear_y = y - ((self.L / 2) * sin(yaw))

        dist = 0
        while dist <= Lf and i < len(path):
            lx = path[i][0]
            ly = path[i][1]
            dist = self.euc_distance((x, y), (lx, ly))
            i += 1

        # Original Pure Pursuit
        alpha = atan2(ly - rear_y, lx - rear_x) - yaw
        pp_angle = atan2(2.0 * self.L * sin(alpha) / Lf, 1.0)

        curvature_control = 0.1*self.k_curva*self.cur_curvature[0] \
            + 0.15*self.k_curva*self.cur_curvature[1] \
            + 0.2*self.k_curva*self.cur_curvature[2] \
            + 0.35*self.k_curva*self.cur_curvature[3] \
            + 0.2*self.k_curva*self.cur_curvature[4]

        curvature_control = abs(curvature_control)
        if pp_angle >= 0:
            angle = degrees(pp_angle) + curvature_control
        elif pp_angle < 0:
            angle = degrees(pp_angle) - curvature_control

        # Set min/max
        angle = max(min(angle, 35.0), -35.0)

        return angle, (lx, ly)

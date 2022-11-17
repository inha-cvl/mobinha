import math
from math import atan2
import numpy as np
from scipy.interpolate import interp1d
from numpy.linalg import norm


class QuadraticSplineInterpolate:
    def __init__(self, x, y):
        self.s = self.calc_s(x, y)
        self.sx = interp1d(self.s, x, fill_value="extrapolate")
        self.sy = interp1d(self.s, y, fill_value="extrapolate")

    def calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        #self.ds = [norm(idx, idy) for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_d(self, sp, x):
        dx = 1.0
        dp = sp(x+dx)
        dm = sp(x-dx)
        d = (dp - dm) / dx
        return d

    def calc_dd(self, sp, x):
        dx = 2.0
        ddp = self.calc_d(sp, x+dx)
        ddm = self.calc_d(sp, x-dx)
        dd = (ddp - ddm) / dx
        return dd

    def calc_yaw(self, s):
        dx = self.calc_d(self.sx, s)
        dy = self.calc_d(self.sy, s)
        yaw = atan2(dy, dx)
        return yaw

    def calc_position(self, s):
        x = self.sx(s)
        y = self.sy(s)
        return x, y

    def calc_curvature(self, s):
        dx = self.calc_d(self.sx, s)
        ddx = self.calc_dd(self.sx, s)
        dy = self.calc_d(self.sy, s)
        ddy = self.calc_dd(self.sy, s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k


def interpolate(points, precision):
    def filter_same_points(points):
        filtered_points = []
        pre_pt = None

        for pt in points:
            if pre_pt is None or pt != pre_pt:
                filtered_points.append(pt)

            pre_pt = pt

        return filtered_points

    points = filter_same_points(points)
    if len(points) < 2:
        return points, None, None, None

    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))

    itp_points = []
    s = []
    yaw = []
    k = []

    for n, ds in enumerate(np.arange(0.0, itp.s[-1], precision)):
        s.append(ds)
        x, y = itp.calc_position(ds)
        dyaw = itp.calc_yaw(ds)

        dk = itp.calc_curvature(ds)

        itp_points.append((float(x), float(y)))
        yaw.append(dyaw)
        k.append(dk)

    return itp_points, s, yaw, k
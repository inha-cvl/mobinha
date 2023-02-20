import numpy as np
import math

#from libs.quintic_polynomials_planner import QuinticPolynomial


class QuarticPolynomial:
    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
            self.a3 * t ** 3 + self.a4 * t ** 4
        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3
        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2
        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t
        return xt


class FrenetPath:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

        self.last_s = -1


def calc_frenet_paths(csp, c_speed, c_d, c_d_d, c_d_dd, s0):
    DT = 0.2  # time tick [s]
    MIN_T = 3.0  # min prediction time [m]
    MAX_T = 7.0  # max prediction time [m]

    c_speed = max(c_speed, 10.0 / 3.6)

    frenet_paths = []

    # Lateral motion planning
    for Ti in np.arange(MIN_T, MAX_T, DT):
        fp = FrenetPath()

        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, 0.0, 0.0, 0.0, Ti)

        fp.t = [t for t in np.arange(0.0, Ti, DT)]
        fp.d = [lat_qp.calc_point(t) for t in fp.t]
        fp.s = [s0 + c_speed * t for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]
        fp.cf = sum(np.power(fp.d_ddd, 2))  # square of jerk

        frenet_paths.append(fp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:
        # calc global positions
        for i, s in enumerate(fp.s):
            ix, iy = csp.calc_position(s)
            if ix is None:
                fp.cf = float("inf")
                break

            i_yaw = csp.calc_yaw(s)
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)
            fp.last_s = s

    return fplist


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd):
    fplist = calc_frenet_paths(csp, c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)

    # find minimum cost path
    min_cost = float("inf")
    selected = None
    local_paths = []
    for n, fp in enumerate(fplist):
        if min_cost > fp.cf:
            min_cost = fp.cf
            selected = n
        local_paths.append(fp)

    return local_paths, selected

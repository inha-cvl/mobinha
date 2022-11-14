from lib import frenet_optimal_trajectory as Frenet
import get_shp
import matplotlib.pyplot as plt
import numpy as np

def test():
    # x = [i for i in range(0,100)]
    # y = [i for i in range(0,200,2)]
    x, y = get_shp.get_coord()

    tx, ty, tyaw, tc, csp = Frenet.generate_target_course(x, y)

    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 2.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]

    plot(csp, tx, ty)

def plot(csp, tx, ty):
    plt.subplots(1)
    plt.plot(ty, tx, "xb", label="input")
    # plt.plot(rx, ry, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.show()

if __name__ == '__main__':
    test()
    # get_shp.get_coord()

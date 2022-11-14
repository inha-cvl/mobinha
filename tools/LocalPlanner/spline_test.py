from libs import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np

from shapely.geometry import LineString
import geopandas as gpd


def interpolate_points(geom, distance):  # Input gpd linestring
    if geom.geom_type == 'LineString':
        num_vert = int(round(geom.length / distance))
        if num_vert == 0:
            num_vert = 1
        return LineString(
            [geom.interpolate(float(n) / num_vert, normalized=True)
             for n in range(num_vert + 1)])


def get_coord():
    shpfile = gpd.read_file('./GIS/path.shp')  # read shp
    shpfile = shpfile.to_crs('epsg:32652')
    shpfile = interpolate_points(shpfile.geometry[0], 5)
    path = shpfile.coords

    X, Y = [], []
    for x, y in path:
        X.append(x)
        Y.append(y)
    return X, Y
    return path

def save(path):
    import pickle
    with open('path.pkl', 'wb') as f:
        pickle.dump(path, f)

def test():
    import random
    # x = [i for i in range(0, 100)]
    # y = [random.randint(0, i) for i in range(0, 200, 2)]
    x, y = get_coord()

    sp = cubic_spline_planner.Spline(x, y)
    csp = cubic_spline_planner.Spline2D(x, y)

    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix - 288350)
        ry.append(iy - 4143900)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    # save([(rx[i], ry[i]) for i in range(len(rx))])

    plt.subplots(1)
    plt.plot(x, y, "xb", label="input")
    plt.plot(rx, ry, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.show()


if __name__ == '__main__':
    test()
    # get_coord_tuple()

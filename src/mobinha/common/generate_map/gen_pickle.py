from libs import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np

from shapely.geometry import LineString
import geopandas as gpd
import utm
import pymap3d


def get_coord():
    shpfile = gpd.read_file('../weekdays/local/Local_inside.shp')  # read shp
    # shpfile = gpd.read_file('../weekdays/start_right.shp')  # generate start
    # shpfile = gpd.read_file('../weekdays/local/Local_outside.shp')  # generate testlink
    path = LineString(shpfile.geometry[0]).coords

    X, Y = [], []
    for x, y in path:
        out = pymap3d.geodetic2enu(
            y, x, 0, 37.5833501770936, 126.88227370105241, 0)
        X.append(out[0])
        Y.append(out[1])
    return X, Y
    return path


def save(path):
    import pickle
    # with open('start_right.pkl', 'wb') as f: #generate start
    # with open('outside.pkl', 'wb') as f: #generate test_link
    with open('inside.pkl', 'wb') as f:
        pickle.dump(path, f)


def generate():
    x, y = get_coord()

    sp = cubic_spline_planner.Spline(x, y)
    csp = cubic_spline_planner.Spline2D(x, y)

    s = np.arange(0, csp.s[-1], 1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    save([[rx[i], ry[i]] for i in range(len(rx))])


if __name__ == '__main__':
    generate()

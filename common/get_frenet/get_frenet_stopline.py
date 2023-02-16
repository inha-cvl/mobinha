from libs import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np

from shapely.geometry import LineString
from scipy.spatial import KDTree
import geopandas as gpd
import pymap3d

def get_coord():
    shpfile = gpd.read_file('../weekdays/local/Local_inside.shp')  # read shp
    path = LineString(shpfile.geometry[0]).coords

    X, Y = [], []
    for x, y in path:
        out = pymap3d.geodetic2enu(y, x, 0, 37.5833501770936, 126.88227370105241, 0)
        X.append(out[0])
        Y.append(out[1])
    return X, Y
    return path

def get_stopline():
    stopfile = gpd.read_file('../weekdays/stoplines/stop_with_right.shp')  # read shp
    lineX, lineY = [], [] 

    for i in stopfile.geometry:
        if i is not None:
            for j in i.coords:
                out = pymap3d.geodetic2enu(j[1], j[0], 0, 37.5833501770936, 126.88227370105241, 0)
                lineX.append(out[0])
                lineY.append(out[1])
    return lineX, lineY

def generate():
    x, y = get_coord()
    line_x, line_y = get_stopline()
    stoplines = set()

    sp = cubic_spline_planner.Spline(x, y)
    csp = cubic_spline_planner.Spline2D(x, y)

    s = np.arange(0, csp.s[-1], 0.5)

    rx, ry, ryaw, rk = [], [], [], []
    lx, ly = [], [] 
    wp = []
    TARGET = [0,0]
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        wp.append([ix,iy])
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    waypoint = KDTree(wp)

    def distance(x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def get_frenet(idx):
        TARGET = [line_x[idx], line_y[idx]]
        point = waypoint.query([TARGET[0], TARGET[1]], 1)[1]
        dist = 9999
        TARGET = np.array(TARGET)

        # frenet_s = 0
        # for i in range(1,point):
            # frenet_s += distance(wp[i][0], wp[i][1], wp[i+1][0], wp[i+1][1])

        return point/2
    val = 0
    stop = []
    for idx in range(len(line_x)):
        s_value = get_frenet(idx)
        stoplines.add(s_value)

    for s_value in sorted(stoplines):
        if s_value - val > 5:
            val = s_value
            stop.append(s_value - 3)

    print(stop)
    print(len(stop))

if __name__ == '__main__':
    # get_coord()
    # print(get_stopline())
    generate()

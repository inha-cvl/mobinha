from libs import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np

from shapely.geometry import LineString
from scipy.spatial import KDTree
import geopandas as gpd
import pymap3d

def get_coord():
    shpfile = gpd.read_file('./weekdays/path.shp')  # read shp
    path = LineString(shpfile.geometry[0])
    path = path.coords

    X, Y = [], []
    for x, y in path:
        out = pymap3d.geodetic2enu(y, x, 0, 37.5833501770936, 126.88227370105241, 0)
        X.append(out[0])
        Y.append(out[1])
    return X, Y
    return path

def generate():
    x, y = get_coord()

    sp = cubic_spline_planner.Spline(x, y)
    csp = cubic_spline_planner.Spline2D(x, y)

    s = np.arange(0, csp.s[-1], 0.5)

    rx, ry, ryaw, rk = [], [], [], []
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


    def onclick(event):
        TARGET = [event.xdata, event.ydata]
        point = waypoint.query([TARGET[0], TARGET[1]], 1)[1]
        plt.plot(TARGET[0],TARGET[1], "xb")
        plt.plot(rx[point], ry[point], "o")
        event.canvas.draw()
        dist = 9999
        TARGET = np.array(TARGET)

        n_x = wp[point][0] - wp[point-1][0]
        n_y = wp[point][1] - wp[point-1][1]
        x_x = TARGET[0] - wp[point-1][0]
        x_y = TARGET[1] - wp[point-1][1]
        
        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y

        frenet_d = distance(x_x,x_y,proj_x,proj_y)

        center_x = 900 - wp[point-1][0]
        center_y = -500 - wp[point-1][1]
        distToPos = distance(center_x, center_y, x_x, x_y)
        distToRef = distance(center_x, center_y, proj_x, proj_y)

        if(distToPos > distToRef):
            frenet_d *= -1

        frenet_s = 0
        for i in range(1,point):
            frenet_s += distance(wp[i][0], wp[i][1], wp[i+1][0], wp[i+1][1])

        print(frenet_d, frenet_s, point)

    fig, ax = plt.subplots()
    plt.plot(rx, ry, "xb", label="waypoints")
    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    plt.show()


if __name__ == '__main__':
    # get_coord()
    generate()

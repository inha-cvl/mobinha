from libs import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np

from shapely.geometry import LineString
from scipy.spatial import KDTree
import geopandas as gpd
import pymap3d

def get_coord():
    shpfile = gpd.read_file('../weekdays/local/Local_outside.shp')  # read shp
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
    # stopfile = gpd.read_file('./stoplines/stop_with_right.shp')  # read shp
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
    lineX, lineY = get_stopline()

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

    # for line in len(lineX)

    waypoint = KDTree(wp)

    def distance(x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def getXY(s,d):
        x,y = csp.calc_position(s)
        yaw = csp.calc_yaw(s)

        x = x + d*np.cos(yaw)
        y = y + d*np.sin(yaw)

        return x,y

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

        qx, qy = getXY(frenet_s, frenet_d)

        plt.plot(qx,qy,"Xr")
        print(frenet_s)

    fig, ax = plt.subplots()
    plt.plot(rx, ry, "xb", label="waypoints")
    plt.plot(lineX, lineY, "go")
    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    plt.show()


if __name__ == '__main__':
    # get_coord()
    # get_stopline()
    generate()

import numpy as np
import math


class ObstacleUtils:
    def distance(x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def object2enu(odom, obj_local_y, obj_local_x):
        rad = odom["yaw"] * (math.pi / 180.0)

        nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
        ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

        obj_x = odom["x"] + ny
        obj_y = odom["y"] + nx

        return obj_x, obj_y

    def object2frenet(local_point, local_path, obj_x, obj_y):  # Array in Frenet Array Out
        target = [obj_x, obj_y]
        point = local_point.query(target, 1)[1]
        if(point == 0):
            return 0, 1000
        wp = local_path

        n_x = wp[point][0] - wp[point-1][0]
        n_y = wp[point][1] - wp[point-1][1]
        x_x = target[0] - wp[point-1][0]
        x_y = target[1] - wp[point-1][1]

        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y

        frenet_d = ObstacleUtils.distance(x_x, x_y, proj_x, proj_y)

        center_x = 900 - wp[point-1][0]
        center_y = -100 - wp[point-1][1]
        distToPos = ObstacleUtils.distance(center_x, center_y, x_x, x_y)
        distToRef = ObstacleUtils.distance(center_x, center_y, proj_x, proj_y)

        if(distToPos > distToRef):
            frenet_d *= -1

        return int(point/2), frenet_d

import numpy as np
import math


class ObstacleUtils:
    def distance(x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def object2enu(odom, obj_local_x, obj_local_y):
        rad = np.radians(odom[2])

        nx = math.cos(rad) * obj_local_x - math.sin(rad) * obj_local_y
        ny = math.sin(rad) * obj_local_x + math.cos(rad) * obj_local_y

        obj_x = odom[0] + nx
        obj_y = odom[1] + ny

        return obj_x, obj_y

    def object2frenet(local_point, local_path, target):  # Array in Frenet Array Out
        point = local_point.query(target, 1)[1]
        if(point == 0):
            return 0, 1000
        wp = local_path

        n_x = wp[point][0] - wp[point-1][0]
        n_y = wp[point][1] - wp[point-1][1]
        x_x = target[0] - wp[point-1][0]
        x_y = target[1] - wp[point-1][1]

        if (n_x*n_x+n_y*n_y) > 0:
            proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        else:
            proj_norm = 0
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y
        
        frenet_s = point
        frenet_d = ObstacleUtils.distance(x_x, x_y, proj_x, proj_y)

        normal_x, normal_y = n_y, -n_x
        dot_product = normal_x * x_x + normal_y * x_y
        if dot_product < 0:
            frenet_d *= -1

        return frenet_s, frenet_d

    def calculate_avoid_gain(obs_d, obs_width, obs_speed):
        half_width = obs_width / 2

        # Adjust the obs_d value based on its sign
        if obs_speed < 10: # 10km/h
            if obs_d < 0:
                adjusted_obs_d = obs_d + half_width
                if -1.3 < adjusted_obs_d < -0.5:
                    return 0.3  # move the car by 30cm to the right
            else:
                adjusted_obs_d = obs_d - half_width
                if 0.5 < adjusted_obs_d < 1.3:
                    return -0.3  # move the car by 30cm to the left
        #TODO: -0.5~0.5 is need STOP

        return 0.0  # no need to move
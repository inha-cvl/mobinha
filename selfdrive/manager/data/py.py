import numpy as np
import math



def object2enu1(odom, obj_local_x, obj_local_y):
    rad = np.radians(odom[2])

    nx = math.cos(-rad) * obj_local_y - math.sin(-rad) * obj_local_x
    ny = math.sin(-rad) * obj_local_y + math.cos(-rad) * obj_local_x

    obj_x = odom[0] + nx
    obj_y = odom[1] + ny

    return obj_x, obj_y

def object2enu(odom, obj_local_x, obj_local_y):
    rad = np.radians(odom[2])

    nx = math.cos(rad) * obj_local_x - math.sin(rad) * obj_local_y
    ny = math.sin(rad) * obj_local_x + math.cos(rad) * obj_local_y

    obj_x = odom[0] + nx
    obj_y = odom[1] + ny

    return obj_x, obj_y

def object2enu2(odom, obj_local_y, obj_local_x):
    rad = odom[2] * (math.pi / 180.0)

    nx = math.cos(-rad) * obj_local_x - math.sin(-rad) * obj_local_y
    ny = math.sin(-rad) * obj_local_x + math.cos(-rad) * obj_local_y

    obj_x = odom[0] + ny
    obj_y = odom[1] + nx

    return obj_x, obj_y
print(object2enu1((-1,2,90),50,0))
print(object2enu((-1,2,90),50,0))
print(object2enu2((-1,2,90),50,0))
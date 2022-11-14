import numpy as np

getFrenet(x, y, maps):
    n_x = wp[point][0] - wp[point-1][0]
    n_y = wp[point][1] - wp[point-1][1]
    x_x = TARGET[0] - wp[point-1][0]
    x_y = TARGET[1] - wp[point-1][1]

    proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
    proj_x = proj_norm*n_x
    proj_y = proj_norm*n_y

    frenet_d = distance(x_x,x_y,proj_x,proj_y)


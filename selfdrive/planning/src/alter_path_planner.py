from numpy.linalg import norm       
import numpy as np
import math
from math import atan2
from scipy.interpolate import interp1d
from cubic_spline_planner import Spline2D

class AlterPathPlanner:
    def __init__(self):
        pass

    def euc_distance(self, pt1, pt2):
        return norm([pt2[0] - pt1[0], pt2[1] - pt1[1]]) 

    def find_nearest_idx(self, pts, pt):
        min_dist = float('inf')
        min_idx = 0
        for idx, pt1 in enumerate(pts):
            dist = self.euc_distance(pt1, pt)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        return min_idx, min_dist
   
    def filter_same_points(self, points):
        filtered_points = []
        pre_pt = None

        for pt in points:
            if pre_pt is None or pt != pre_pt:
                filtered_points.append(pt)    

            pre_pt = pt

        return filtered_points


    def cubic_spline(self, ref_path, precision=0.5):
        x_list, y_list = zip(*ref_path)
        sp = Spline2D(x_list, y_list)

        ds=precision # [m] distance of each intepolated points
        s = np.arange(0, sp.s[-1], ds)
        splined = []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            splined.append((ix, iy))
        return splined
 
    def run(self, ego_pos, obs_pos, adj_lanelet, straight_step=10, back_step=3, precision=0.5, p=0.7):
        curve_end_idx, _ = self.find_nearest_idx(adj_lanelet, obs_pos)
        straight_end_idx = curve_end_idx + straight_step

        curve_start_pt = ego_pos
        curve_end_pt =  adj_lanelet[curve_end_idx]

        tmp_x = p*curve_start_pt[0] + (1-p)*curve_end_pt[0]
        tmp_y = p*curve_start_pt[1] + (1-p)*curve_end_pt[1]
        curve_mid1_pt = (tmp_x, tmp_y)

        curve_mid2_pt = adj_lanelet[curve_end_idx-back_step]
        ref_points = [curve_start_pt, curve_mid1_pt, curve_mid2_pt, curve_end_pt] #(x, y)

        curve_path = self.cubic_spline(ref_points, precision)
       
        straight_path = adj_lanelet[curve_end_idx+1 : straight_end_idx]
        alter_path = curve_path + straight_path
        return alter_path, straight_end_idx, ref_points
       
if __name__ == '__main__':

    ego_pos = (0, 0)
    obs_pos = (0, 10)
    
    adj_lanelet = [(3,i) for i in range(30)]
    straight_step = 10
    back_step = 3
    precision=0.5
    p=0.7
    # main
    app = AlterPathPlanner()
    alter_path, end_idx, ref_points = app.run(ego_pos, obs_pos, adj_lanelet, \
                                            straight_step, back_step, precision, p)
    # end main


    print(alter_path)
    import matplotlib.pyplot as plt
    plt.axis('equal')
    x2 =  [adj_lanelet[i][0] for i in range(len(adj_lanelet))]
    y2 =  [adj_lanelet[i][1] for i in range(len(adj_lanelet))]
    plt.plot(x2, y2)

    x1 = [alter_path[i][0] for i in range(len(alter_path)) ]
    y1 = [alter_path[i][1] for i in range(len(alter_path)) ]
    plt.scatter(x1, y1, s=5)

    for pt in ref_points:
        plt.plot(pt[0], pt[1], 'o')

    plt.scatter(obs_pos[0], obs_pos[1],  s=300)
    plt.show()

#left_id = lanelets[left_id]['adjacentLeft']
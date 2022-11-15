from shapely.geometry import Polygon
from shapely.geometry import Point as GPoint
import matplotlib.patches as patches
from numpy import log, min
import rospy
def euc_distance(pt1, pt2):
    return norm([pt2[0] - pt1[0], pt2[1] - pt1[1]]) 


def new_velocity_plan(ax, drawn, st_param, ref_v, last_s, s, max_v, cur_v, cur_a, tl_s, tl_time, tl_state, objects_s): #, objects_s=None, rel_v=None):
    KPH_TO_MPS = 1 / 3.6
    MPS_TO_KPH = 3.6
    #print('tl_s', tl_s)
    ref_v *= KPH_TO_MPS
    #calculate d : distance from the obstacle
    d_list = []

    traffic_d = tl_s
    margin = 5
    d_list.append(traffic_d)
    

    d_list.append(last_s-s)

    rospy.loginfo(str(last_s - s))
    
    for object in objects_s:
        for obj in object:
            #d = euc_distance((0,0), (obj.x, obj.y))
            d = obj[1] - s #obj[0]: s in s-d coordinate
            d_list.append(d)
        
    min_d = min(d_list)

    if not len(d_list)==0:
        if min_d > 50:
            target_v = ref_v #max speed in map
        else:
            #calculate target_v
            k = 0.8 # [m/s]
            target_v = k*log(min_d - margin + 1)
    else:
        target_v = ref_v
    
    #original
    if drawn is not None:
        for d in drawn:
            d.remove()

    drawn = []
    obstacles = []
    s_min, s_max, t_max = st_param['s_min'], st_param['s_max'], st_param['t_max']
    dt, dt_exp = st_param['dt'], st_param['dt_exp']
    last = [(0.0, last_s-2.0), (t_max, last_s-2.0),(t_max, last_s+5.0), (0.0, last_s+5.0)]
    obstacles.append(last)
    d = ax.add_patch(patches.Polygon([pt for pt in last], closed=True, edgecolor='black', facecolor='gray'))
    drawn.append(d)
    ####
    

    return target_v*MPS_TO_KPH, drawn
import time
tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0}
def timer(sec):
    if time.time() - tick[sec] > sec:
        tick[sec] = time.time()
        return True
    else:
        return False

def alive_counter(alv_cnt):
    alv_cnt += 1
    alv_cnt = 0 if alv_cnt > 255 else alv_cnt
    return alv_cnt

import numpy as np
def calc_idx(path, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(path):
        dist = np.sqrt((pt[0]-pt1[0])**2+(pt[1]-pt1[1])**2)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    if min_idx == len(path) - 1:
        pt1 = path[min_idx-1]
    else:
        pt1 = path[min_idx]

    return min_idx

def calculate_cte(path, position):
    try:
        idx = calc_idx(path, position)
        Ax, Ay = path[idx]
        Bx, By = path[idx+1]
        Px, Py = position

        numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
        denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
        cte = numerator / denominator if denominator != 0 else 0

        cross_product = (Bx - Ax) * (Py - Ay) - (By - Ay) * (Px - Ax)
        
        if cross_product > 0:
            return idx, -cte
        elif cross_product < 0:
            return idx, cte
        else:
            return idx, 0
    except:
        return idx, 0

def limit_steer_change(current_v, prev_steer, current_steer):
    # saturation_th = 20
    saturation_th = -5/9*(current_v*3.6-6)+15
    saturation_th = np.clip(saturation_th, 2, 20)
    # print("                         ", saturation_th)
    saturated_steering_angle = current_steer
    diff = max(min(current_steer-prev_steer, saturation_th), -saturation_th)
    saturated_steering_angle = prev_steer + diff
    return saturated_steering_angle
    
import rospy
import sys
def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Exiting gracefully...')
    rospy.signal_shutdown('Exiting')
    sys.exit(0)
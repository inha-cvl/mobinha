from planner_utils import *
import time 

def eventCheck(future_ids, event_dict): #for testing
    event = None
    cnt = 0
    for _, e_id in event_dict.items():
        cnt += 1
        if e_id in future_ids:
            event = cnt
            #1: 'Car1', 2: 'Car2', 3: 'Pedestrian1', 4: 'Pedestrian2', 5: 'Bump',6: 'Accident', ( real )
    return event

def waitProcess(wait_event, car_v, start_time):
    wait_time = 0
    if car_v == 0:
        if not wait_event :
            start_time = time.time()
        wait_time = time.time()-start_time
        if( wait_time > 0 ):
            wait_event = True
        elif(wait_time > 10):
            wait_event = False
    return wait_event, start_time


def processingEvent1(wait_event, car_v, start_time):
    return waitProcess(wait_event, car_v, start_time)

    

def processingEvent2(tmap, future_idx, future_pts, shortest_path):
    offset = 5
    lane_change_path = shortest_path
    adjacent = lanelet_matching_adj(tmap.tiles, tmap.tile_size, future_pts[offset-1])
    adj_idx = 1 if adjacent[1] != None else 0
    cnt = 0
    for i in range(len(shortest_path)):
        if (future_idx-offset)<= i and i<future_idx+offset:
            lane_change_path[i] = lanelet_matching_adj(tmap.tiles, tmap.tile_size, future_pts[cnt])[adj_idx]
            cnt += 1
    return lane_change_path

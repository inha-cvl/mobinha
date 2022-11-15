from inspect import currentframe
from planner_utils import *


def fetchStopLine(current_id):
    stop_line_ids = ["73_8","31_88","82_85","81_90","24_90","40_3","77_11","80_90","79_90","78_90","39_90","60_3","59_6","57_99","75_99"]
    
    print(current_id)
    for stop_line_id in stop_line_ids:
        sid = stop_line_id.split('_')
        cid = int(current_id.split('_')[0])
        if(int(sid[0]) == cid):
            rest = abs(int(sid[1])-int(current_id.split('_')[1]))
            return 0.5*rest
    return 300
    
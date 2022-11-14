import math

import rospy
from visualization_msgs.msg import MarkerArray

from config.config import Config
from lanelet_map import LaneletMap
from viz import *


if __name__ == "__main__":
    rospy.init_node('construction_sites', anonymous=False)

    config = Config()
    lmap = LaneletMap(config.map_path)

    pub_construction_sites = rospy.Publisher('/construction_sites', MarkerArray, queue_size=1, latch=True)

    construction_sites = []
    for id_, s_idx, e_idx in config.construction_sites:
        waypoints = lmap.lanelets[id_]['waypoints']
        yaws = lmap.lanelets[id_]['yaw']
        pts = []
        pts.append((waypoints[s_idx][0] + 1.5 * math.sin(yaws[s_idx]), waypoints[s_idx][1] - 1.5 * math.cos(yaws[s_idx])))
        pts.append((waypoints[e_idx][0] + 1.5 * math.sin(yaws[e_idx]), waypoints[e_idx][1] - 1.5 * math.cos(yaws[e_idx])))
        pts.append((waypoints[e_idx][0] - 1.5 * math.sin(yaws[e_idx]), waypoints[e_idx][1] + 1.5 * math.cos(yaws[e_idx])))
        pts.append((waypoints[s_idx][0] - 1.5 * math.sin(yaws[s_idx]), waypoints[s_idx][1] + 1.5 * math.cos(yaws[s_idx])))
        pts.append((waypoints[s_idx][0] + 1.5 * math.sin(yaws[s_idx]), waypoints[s_idx][1] - 1.5 * math.cos(yaws[s_idx])))
        construction_sites.append(pts)

    construction_sites_viz = ConstructionSiteViz(construction_sites)
    pub_construction_sites.publish(construction_sites_viz)

    rospy.spin()
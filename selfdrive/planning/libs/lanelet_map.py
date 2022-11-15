import json
import numpy as np

import rospy
from visualization_msgs.msg import MarkerArray

from selfdrive.visualize.viz import *


class LaneletMap:
    def __init__(self, map_path):
        with open(map_path, 'r') as f:
            map_data = json.load(f)
        self.lanelets = map_data['lanelets']
        self.groups = map_data['groups']
        self.precision = map_data['precision']
        self.for_viz = map_data['for_vis']

        # update base lla
        self.basella = map_data['base_lla']

        rospy.set_param('base_lla', map_data['base_lla'])

        self.pub_lanelet_map = rospy.Publisher(
            '/lanelet_map', MarkerArray, queue_size=1, latch=True)

        lanelet_map_viz = LaneletMapViz(self.lanelets, self.for_viz)
        self.pub_lanelet_map.publish(lanelet_map_viz)

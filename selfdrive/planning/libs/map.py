import json
import rospy
from visualization_msgs.msg import MarkerArray
import sys
from os import path

if __package__ is None:
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    from visualize.viz import *
else:
    sys.path.append(path.dirname(
        (path.dirname(path.dirname(path.abspath(__file__))))))
    from visualize.viz import *


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


class TileMap:
    def __init__(self, lanelets, tile_size):
        self.tiles = {}
        self.tile_size = tile_size

        for id_, data in lanelets.items():
            for n, (x, y) in enumerate(data['waypoints']):
                row = int(x // tile_size)
                col = int(y // tile_size)
                if self.tiles.get((row, col)) is None:
                    self.tiles[(row, col)] = {}

                if self.tiles[(row, col)].get(id_) is None:
                    self.tiles[(row, col)][id_] = {'waypoints': [], 'idx': []}

                self.tiles[(row, col)][id_]['waypoints'].append((x, y))
                self.tiles[(row, col)][id_]['idx'].append(n)

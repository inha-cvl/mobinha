import json
import rospy


class LaneletMap:
    def __init__(self, map_path):
        with open(map_path, 'r') as f:
            map_data = json.load(f)
        self.map_data = map_data
        self.lanelets = map_data['lanelets']
        self.groups = map_data['groups']
        self.precision = map_data['precision']
        self.for_viz = map_data['for_vis']
        self.basella = map_data['base_lla']


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

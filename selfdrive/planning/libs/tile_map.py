

class TileMap:
    def __init__(self, lanelets, tile_size):
        self.tiles = {}
        self.tile_size = tile_size
        
        for id_, data in lanelets.items():
            for n, (x, y) in enumerate(data['waypoints']):
                row = int(x // tile_size)
                col = int(y // tile_size)
                if self.tiles.get((row,col)) is None:
                    self.tiles[(row,col)] = {}

                if self.tiles[(row,col)].get(id_) is None:
                    self.tiles[(row,col)][id_] = {'waypoints':[], 'idx':[]}

                self.tiles[(row,col)][id_]['waypoints'].append((x, y))
                self.tiles[(row,col)][id_]['idx'].append(n)
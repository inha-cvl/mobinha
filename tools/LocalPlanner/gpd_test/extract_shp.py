import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import os
import geopandas as gpd
from shapely import wkt
from shapely.geometry import LineString

def interpolate_points(geom, distance):
    if geom.geom_type == 'LineString':
        num_vert = int(round(geom.length / distance))
        if num_vert == 0:
            num_vert = 1
        return LineString(
            [geom.interpolate(float(n) / num_vert, normalized=True)
             for n in range(num_vert + 1)])

# multiline_r = redistribute_vertices(multiline, 100)

if __name__ == '__main__':
    shpfile = gpd.read_file('./GIS/F.shp')
    print([_ for _ in shpfile.geometry[1].coords])
    # print(shpfile.apply(lambda m : [x in m['id']]))

    pass

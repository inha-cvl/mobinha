import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import os
import geopandas as gpd
import pymap3d as pm
import utm
from shapely import wkt
from shapely.geometry import LineString

def interpolate_points(geom, distance): #Input gpd linestring
    if geom.geom_type == 'LineString':
        # print(geom.length)
        num_vert = int(round(geom.length / distance))
        if num_vert == 0:
            num_vert = 1
        return LineString(
            [geom.interpolate(float(n) / num_vert, normalized=True)
             for n in range(num_vert + 1)])

if __name__ == '__main__':
    shpfile = gpd.read_file('./GIS/F.shp') #read shp
    shpfile = shpfile.to_crs('epsg:32652')
    new = interpolate_points(shpfile.geometry[1], 5)

    F = [i for i in shpfile.geometry[1].coords] #Coord to list
    # print(utm.from_latlon(F[0][1], F[0][0]))
    
    # print(shpfile.apply(lambda m : [x in m['id']]))

from libs import cubic_spline_planner
from shapely.geometry import LineString
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np

def interpolate_points(geom, distance): #Input gpd linestring
    if geom.geom_type == 'LineString':
        # print(geom.length)
        num_vert = int(round(geom.length / distance))
        if num_vert == 0:
            num_vert = 1
        return LineString(
            [geom.interpolate(float(n) / num_vert, normalized=True)
             for n in range(num_vert + 1)])

def get_coord_tuple():
    shpfile = gpd.read_file('./GIS/B.shp') #read shp
    shpfile = shpfile.to_crs('epsg:32652')
    new = interpolate_points(shpfile.geometry[0], 5)
    return [[x - 288350, y - 288350] for x, y in new.coords]
    # return new.coords

def get_coord():
    shpfile = gpd.read_file('./GIS/B.shp') #read shp
    shpfile = shpfile.to_crs('epsg:32652')
    new = interpolate_points(shpfile.geometry[1], 5)
    X, Y = [], []
    for x, y in new.coords:
        X.append(x - 288350)
        Y.append(y - 4143900)
    return X, Y

if __name__ == '__main__':
    # print(get_coord_tuple)
    print(get_coord())

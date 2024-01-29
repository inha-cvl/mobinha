import geopandas as gpd
import pymap3d
import matplotlib.pyplot as plt
import utm
import numpy as np
from matplotlib.lines import Line2D

def rotate_points(x, y, angle):
    angle_rad = np.radians(angle)  
    x = np.array(x)  
    y = np.array(y)  
    x_rotated = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rotated = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_rotated, y_rotated

class PathAnalysis:
    def __init__(self, map_name, base_lla, rotation_angle):
        self.map = map_name
        self.base_lla = base_lla
        self.rotation_angle = rotation_angle
        self.is_utm = True
    
    def to_cartesian(self, tx, ty, alt=None):
        if self.is_utm:
            lat, lon = utm.to_latlon(tx, ty, 52, 'N')
        else:
            lat, lon = ty, tx
        if alt is None:
            x, y, _ = pymap3d.geodetic2enu(lat, lon, self.base_lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y
        else:
            x, y, z = pymap3d.geodetic2enu(lat, lon, alt, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y, z

    def process_surface_line(self):
        shp_file = '{}_B2_SURFACELINEMARK.shp'.format(self.map)
        gdf = gpd.read_file(shp_file)
        lines_x, lines_y = [], []
        for index, row in gdf.iterrows():
            if row['geometry'].geom_type == 'LineString':
                line_x, line_y = [], []
                for point in row['geometry'].coords:
                    lon, lat = point[0], point[1]
                    x, y = self.to_cartesian(lon, lat)
                    line_x.append(x)
                    line_y.append(y)
                rotated_x, rotated_y = rotate_points(np.array(line_x), np.array(line_y), self.rotation_angle)
                lines_x.append(rotated_x)
                lines_y.append(rotated_y)
        self.surface_lines = (lines_x, lines_y)

    def plot_data(self):
        # Plot surface lines
        for line_x, line_y in zip(*self.surface_lines):
            plt.plot(line_x, line_y, color='black', linewidth=0.5)
            
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.grid(False)
        plt.xticks([])
        plt.yticks([])
        plt.show()

#!---
map = 'kcity'

if map == 'songdo':
    base_lla = (37.3888319, 126.6428739, 7.369)
    rotation_angle = 0
elif map == 'kcity':
    base_lla = (37.2292221592864, 126.76912499027308, 29.18400001525879)
    rotation_angle = 0

analyzer = PathAnalysis(map, base_lla, rotation_angle)

analyzer.process_surface_line()


# Plot data
analyzer.plot_data()
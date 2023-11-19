import geopandas as gpd
import pymap3d
import matplotlib.pyplot as plt
import utm
import numpy as np
import json
import rosbag
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
        self.gt_coords = self.load_gt_coords()
        self.vehicle_data = {}
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
        
    def load_gt_coords(self):
        with open('{}_path_data.json'.format(self.map), 'r') as file:
            data = json.load(file)
        x_coords = [point[0] for point in data['global_path']]
        y_coords = [point[1] for point in data['global_path']]
        return rotate_points(x_coords, y_coords, self.rotation_angle)

    def process_bag_file(self, bag_file_name):
        bag_path = f'./{self.map}/{bag_file_name}'
        bag = rosbag.Bag(bag_path)
        x_data, y_data = [], []
        for topic, msg, t in bag.read_messages(topics=['/veh_pose']):
            x_data.append(msg.x)
            y_data.append(msg.y)
        bag.close()
        rotated_x, rotated_y = rotate_points(x_data, y_data, self.rotation_angle)
        self.vehicle_data[bag_file_name] = (rotated_x, rotated_y)

    def plot_data(self):
        # Plot surface lines
        for line_x, line_y in zip(*self.surface_lines):
            plt.plot(line_x, line_y, color='black', linewidth=0.5)
            
        # Plot Ground Truth
        gt_x, gt_y = self.gt_coords
        plt.plot(gt_x, gt_y, color='green', linewidth=2, label='Ground Truth')

        # Plot vehicle data
        colors = ['red', 'blue', 'orange', 'purple', 'cyan']
        labels = ['ours', 'stanley', 'w/o accel limit', 'PP', 'w/o curve decel']
        for i, (file_name, (x, y)) in enumerate(self.vehicle_data.items()):
            plt.scatter(x, y, color=colors[i], s=3, label=labels[i])

        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()
        plt.show()

#! Main
map = 'kcity'

if map == 'songdo':
    base_lla = (37.3888319, 126.6428739, 7.369)
    rotation_angle = 45 + 175
elif map == 'kcity':
    base_lla = (37.2292221592864, 126.76912499027308, 29.18400001525879)
    rotation_angle = 0#88

analyzer = PathAnalysis(map, base_lla, rotation_angle)

analyzer.process_surface_line()

# Process each bag file
for bag_file in ['1-2.bag', '2-1.bag', '4-2.bag', '5-1.bag', '6-1.bag']:
    analyzer.process_bag_file(bag_file)

# Plot data
analyzer.plot_data()



# #* surface line
# shp_file = '{}_B2_SURFACELINEMARK.shp'.format(map)
# gdf = gpd.read_file(shp_file)
# lines_x = []
# lines_y = []
# for index, row in gdf.iterrows():
#     if row['geometry'].geom_type == 'LineString':
#         line_x = []
#         line_y = []
#         for point in row['geometry'].coords:
#             lon, lat = point[0], point[1]  # 좌표 추출 (경도, 위도)
#             x, y = transformer.to_cartesian(lon, lat)
#             line_x.append(x)
#             line_y.append(y)
#         lines_x.append(line_x)
#         lines_y.append(line_y)
# for line_x, line_y in zip(lines_x, lines_y):
#     line_x, line_y = rotate_points(np.array(line_x), np.array(line_y), rotation_angle)
#     plt.plot(line_x, line_y, color='black', linewidth=0.5)

# #* ours
# bag = rosbag.Bag('./{}/1-2.bag'.format(map))
# x_data = []
# y_data = []
# for topic, msg, t in bag.read_messages(topics=['/veh_pose']):
#     x_data.append(msg.x)
#     y_data.append(msg.y)
# bag.close()
# veh_x1, veh_y1 = rotate_points(x_data, y_data, rotation_angle)

# #* GT
# with open('{}_path_data.json'.format(map), 'r') as file:
#     data = json.load(file)
# x_coords = [point[0] for point in data['global_path']]
# y_coords = [point[1] for point in data['global_path']]
# gt_x, gt_y = rotate_points(x_coords, y_coords, rotation_angle)


# #* Plot
# plt.scatter(veh_x1, veh_y1, color='red', s=3, label='ours')
# plt.scatter(veh_x2, veh_y2, color='blue', s=3, label='stanley')
# plt.scatter(veh_x4, veh_y4, color='orange', s=3, label='w/o accel limit')
# plt.scatter(veh_x5, veh_y5, color='purple', s=3, label='PP')
# plt.scatter(veh_x6, veh_y6, color='cyan', s=3, label='w/o curve decel')
# plt.plot(gt_x, gt_y, color='green', linewidth=2, label='Ground Truth')

# plt.xlabel('x (m)')
# plt.ylabel('y (m)')
# legend_handles = [
#     Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=7, label='ours'), 
#     Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=7, label='stanley'),
#     Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', markersize=7, label='w/o accel limit'),
#     Line2D([0], [0], marker='o', color='w', markerfacecolor='purple', markersize=7, label='PP'),
#     Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan', markersize=7, label='w/o curve decel'),
#     Line2D([0], [0], color='green', linewidth=2, label='Ground Truth')]

# plt.legend(handles=legend_handles, loc='upper right')
# plt.show()
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

    def filter_data_near_points(self, x_data, y_data, distance_threshold=5):
        # 시작점과 끝점을 정의
        start_point = (self.gt_coords[0][0], self.gt_coords[1][0])
        end_point = (self.gt_coords[0][-1], self.gt_coords[1][-1])

        filtered_x, filtered_y = [], []
        for x, y in zip(x_data, y_data):
            if (self.euclidean_distance((x, y), start_point) <= distance_threshold or
                self.euclidean_distance((x, y), end_point) <= distance_threshold):
                filtered_x.append(x)
                filtered_y.append(y)
        return filtered_x, filtered_y

    def euclidean_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def find_nearest_index(self, data_x, data_y, point):
        distances = [self.euclidean_distance((dx, dy), point) for dx, dy in zip(data_x, data_y)]
        return np.argmin(distances)
    
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
        if self.map == 'songdo':
            x_coords = x_coords[:-400]
            y_coords = y_coords[:-400]

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
        start_point = (self.gt_coords[0][0], self.gt_coords[1][0])
        end_point = (self.gt_coords[0][-1], self.gt_coords[1][-1])
        # 레전드 핸들 생성
        legend_handles = []

        # GT 핸들 추가
        gt_handle = Line2D([0], [0], color='green', linewidth=2, linestyle='-', label='Ground Truth')
        legend_handles.append(gt_handle)
        # Plot vehicle data
        colors = ['red', 'blue', 'orange', 'purple', 'cyan']
        labels = ['Ours', 'Stanley', 'Pure-Pursuit', 'w/o accel-limit', 'w/o path-decel']
        # for i, (file_name, (x, y)) in enumerate(self.vehicle_data.items()):
        #     plt.scatter(x, y, color=colors[i], s=3, label=labels[i])

        for i, (file_name, (x, y)) in enumerate(self.vehicle_data.items()):
            start_index = self.find_nearest_index(x, y, start_point)
            end_index = self.find_nearest_index(x, y, end_point)
            plt.scatter(x[start_index:end_index+1], y[start_index:end_index+1], color=colors[i], s=3, label=labels[i])

        # 나머지 항목들에 대한 핸들 추가
        for color, label in zip(colors, labels):
            # 점 핸들만 추가
            point_handle = Line2D([0], [0], color=color, marker='o', markersize=6, linestyle='', label=label)
            legend_handles.append(point_handle)

        # 레전드 생성
        # plt.legend(handles=legend_handles, loc='upper right')
        # plt.xlabel('x (m)')
        # plt.ylabel('y (m)')
        plt.grid(False)
        plt.xticks([])
        plt.yticks([])
        plt.show()


# map = 'songdo'

# if map == 'songdo':
#     base_lla = (37.3888319, 126.6428739, 7.369)
#     rotation_angle = 45 + 175
# elif map == 'kcity':
#     base_lla = (37.2292221592864, 126.76912499027308, 29.18400001525879)
#     rotation_angle = 88

# analyzer = PathAnalysis(map, base_lla, rotation_angle)

# analyzer.process_surface_line()

# # Process each bag file
# for bag_file in ['1-1.bag', '2-2.bag', '5-1.bag', '4-1.bag', '6-1.bag']:
#     analyzer.process_bag_file(bag_file)

# # Plot data
# analyzer.plot_data()

#!---
map = 'kcity'

if map == 'songdo':
    base_lla = (37.3888319, 126.6428739, 7.369)
    rotation_angle = 45 + 175
elif map == 'kcity':
    base_lla = (37.2292221592864, 126.76912499027308, 29.18400001525879)
    rotation_angle = 88

analyzer = PathAnalysis(map, base_lla, rotation_angle)

analyzer.process_surface_line()

# Process each bag file
for bag_file in ['1-1.bag', '2-3.bag', '5-2.bag', '4-3.bag', '6-2.bag']:
    analyzer.process_bag_file(bag_file)

# Plot data
analyzer.plot_data()
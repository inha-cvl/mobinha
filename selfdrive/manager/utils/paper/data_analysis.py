import numpy as np
import json
import rosbag
from nav_msgs.msg import Odometry
from novatel_oem7_msgs.msg import INSPVA
from sensor_msgs.msg import Imu

def numpy_to_list(data):
    if isinstance(data, np.ndarray):
        return data.tolist()
    elif isinstance(data, list):
        return [numpy_to_list(item) for item in data]
    return data

class PathDataAnalyzer:
    def __init__(self, map_name, start_point_gt, end_point_gt):
        self.map = map_name
        self.gt_coords = self.load_gt_coords()
        self.start_point_gt = start_point_gt
        self.end_point_gt = end_point_gt

    def load_gt_coords(self):
        with open(f'{self.map}_path_data.json', 'r') as file:
            data = json.load(file)
        x_coords = [point[0] for point in data['global_path']]
        y_coords = [point[1] for point in data['global_path']]
        return list(zip(x_coords, y_coords))

    def extract_odom(self, bag_file):
        v_data, a_data, jerk_data, roll_data, pitch_data, timestamps = [], [], [], [], [], []
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['/novatel/oem7/odom', '/gps/imu', '/veh_pose', '/novatel/oem7/inspva']):
            if topic == '/novatel/oem7/odom':
                v_data.append((msg.twist.twist.linear.x, msg.twist.twist.linear.y))
            if topic == '/gps/imu':
                a_data.append((msg.linear_acceleration.x, msg.linear_acceleration.y))
                timestamps.append(msg.header.stamp.to_sec())
            if topic == '/novatel/oem7/inspva':
                roll_data.append(msg.roll)
                pitch_data.append(msg.pitch)
        bag.close()

        for i in range(1, len(a_data)):
            delta_acc = np.array(a_data[i]) - np.array(a_data[i - 1])
            delta_time = timestamps[i] - timestamps[i - 1]
            jerk = delta_acc / delta_time if delta_time > 0 else (0, 0)
            jerk_data.append(jerk)

        return v_data, a_data, jerk_data, roll_data, pitch_data, timestamps

    def find_nearest_index(self, point):
        min_distance = float('inf')
        nearest_index = None

        for index, gt_point in enumerate(self.gt_coords):
            distance = np.sqrt((gt_point[0] - point[0])**2 + (gt_point[1] - point[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_index = index

        return nearest_index
    
    def find_nearest_time(self, target_time, timestamps):
        # 가장 작은 시간 차이와 그 인덱스를 초기화
        min_time_diff = float('inf')
        nearest_time = None

        # 모든 타임스탬프를 순회하며 가장 가까운 시간을 찾음
        for time in timestamps:
            time_diff = abs(time - target_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                nearest_time = time

        return nearest_time

    def process_bag_file(self, bag_file, name):
        bag_path = f'./{self.map}/{bag_file}'
        v_data, a_data, jerk_data, roll_data, pitch_data, timestamps = self.extract_odom(bag_path)

        start_time = self.find_nearest_time(self.start_point_gt, timestamps)
        end_time = self.find_nearest_time(self.end_point_gt, timestamps)

        v_data_extracted = [v for v, t in zip(v_data, timestamps) if start_time <= t <= end_time]
        a_data_extracted = [a for a, t in zip(a_data, timestamps) if start_time <= t <= end_time]
        jerk_data_extracted = [j for j, t in zip(jerk_data, timestamps) if start_time <= t <= end_time]
        roll_data_extracted = [r for r, t in zip(roll_data, timestamps) if start_time <= t <= end_time]
        pitch_data_extracted = [p for p, t in zip(pitch_data, timestamps) if start_time <= t <= end_time]
        
        data_to_save = {
            "velocity": numpy_to_list(v_data_extracted),
            "acceleration": numpy_to_list(a_data_extracted),
            "jerk": numpy_to_list(jerk_data_extracted),
            "roll": numpy_to_list(roll_data_extracted),
            "pitch": numpy_to_list(pitch_data_extracted)
        }

        # start_index = self.find_nearest_index(self.start_point_gt)
        # end_index = self.find_nearest_index(self.end_point_gt)

        # data_to_save = {
        #     "velocity": numpy_to_list(v_data[start_index:end_index + 1]),
        #     "acceleration": numpy_to_list(a_data[start_index:end_index + 1]),
        #     "jerk": numpy_to_list(jerk_data[start_index:end_index + 1]),
        #     "roll": numpy_to_list(roll_data[start_index:end_index + 1]),
        #     "pitch": numpy_to_list(pitch_data[start_index:end_index + 1])
        # }

        print(len(v_data), len(a_data), len(jerk_data), len(roll_data), len(pitch_data))

        with open(f'./{self.map}/{bag_file}_data_{name}.json', 'w') as json_file:
            json.dump(data_to_save, json_file, indent=4)
        
    def process_full_bag_file(self, bag_file):
        bag_path = f'./{self.map}/{bag_file}'
        v_data, a_data, jerk_data, roll_data, pitch_data, timestamps = self.extract_odom(bag_path)

        data_to_save = {
            "velocity": numpy_to_list(v_data),
            "acceleration": numpy_to_list(a_data),
            "jerk": numpy_to_list(jerk_data),
            "roll": numpy_to_list(roll_data),
            "pitch": numpy_to_list(pitch_data)
        }
        print(len(v_data), len(a_data), len(jerk_data), len(roll_data), len(pitch_data))
        with open(f'./{self.map}/{bag_file}_data_full.json', 'w') as json_file:
            json.dump(data_to_save, json_file, indent=4)


map = 'kcity'

start_point_gt = (470,1634)
end_point_gt = (363,1835)
name = 'curve'

analyzer = PathDataAnalyzer(map, start_point_gt, end_point_gt)

for bag_file in ['1-1.bag', '2-1.bag', '4-1.bag', '5-1.bag', '6-1.bag']:
    analyzer.process_bag_file(bag_file, name)
    analyzer.process_full_bag_file(bag_file)
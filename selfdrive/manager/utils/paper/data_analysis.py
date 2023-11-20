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
    def __init__(self, map_name):
        self.map = map_name
        self.gt_coords = self.load_gt_coords()
        self.gt_yaws = self.load_gt_yaws()

    def load_gt_coords(self):
        with open(f'{self.map}_path_data.json', 'r') as file:
            data = json.load(file)
        x_coords = [point[0] for point in data['global_path']]
        y_coords = [point[1] for point in data['global_path']]
        return list(zip(x_coords, y_coords))
    
    def load_gt_yaws(self):
        with open(f'{self.map}_path_data.json', 'r') as file:
            data = json.load(file)
        return data['global_yaw']
    
    def find_nearest_veh_time(self, point, veh_data, veh_timestamps):
        min_distance = float('inf')
        nearest_time = None

        for data, t in zip(veh_data, veh_timestamps):
            distance = np.sqrt((data[0] - point[0])**2 + (data[1] - point[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_time = t

        return nearest_time

    def extract_odom(self, bag_file):
        v_data, a_data, jerk_data, roll_data, pitch_data, veh_data = [], [], [], [], [], []
        v_timestamps, a_timestamps, jerk_timestamps, roll_timestamps, pitch_timestamps, veh_timestamps = [], [], [], [], [], []
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['/novatel/oem7/odom', '/gps/imu', '/veh_pose', '/novatel/oem7/inspva']):
            if topic == '/novatel/oem7/odom':
                v_data.append((msg.twist.twist.linear.x, msg.twist.twist.linear.y))
                v_timestamps.append(t.to_sec())
            if topic == '/gps/imu':
                a_data.append((msg.linear_acceleration.x, msg.linear_acceleration.y))
                a_timestamps.append(t.to_sec())
            if topic == '/novatel/oem7/inspva':
                roll_data.append(msg.roll)
                pitch_data.append(msg.pitch)
                roll_timestamps.append(t.to_sec())
                pitch_timestamps.append(t.to_sec())
            if topic == '/veh_pose':
                veh_data.append((msg.x, msg.y, msg.theta))
                veh_timestamps.append(t.to_sec())
        bag.close()

        for i in range(1, len(a_data)):
            delta_acc = np.array(a_data[i]) - np.array(a_data[i - 1])
            delta_time = a_timestamps[i] - a_timestamps[i - 1]
            jerk = delta_acc / delta_time if delta_time > 0 else (0, 0)
            jerk_data.append(jerk)
            jerk_timestamps.append(a_timestamps[i])  # Jerk의 타임스탬프를 기록

        return {
                "velocity": (v_data, v_timestamps),
                "acceleration": (a_data, a_timestamps),
                "jerk": (jerk_data, jerk_timestamps),
                "roll": (roll_data, roll_timestamps),
                "pitch": (pitch_data, pitch_timestamps),
                "vehicle": (veh_data, veh_timestamps)
                }

    def process_full_bag_file(self, bag_file):
        bag_path = f'./{self.map}/{bag_file}'
        data_dict = self.extract_odom(bag_path)

        data_to_save = {
            "velocity": {"data": numpy_to_list(data_dict["velocity"][0]), "timestamps": numpy_to_list(data_dict["velocity"][1])},
            "acceleration": {"data": numpy_to_list(data_dict["acceleration"][0]), "timestamps": numpy_to_list(data_dict["acceleration"][1])},
            "jerk": {"data": numpy_to_list(data_dict["jerk"][0]), "timestamps": numpy_to_list(data_dict["jerk"][1])},
            "roll": {"data": numpy_to_list(data_dict["roll"][0]), "timestamps": numpy_to_list(data_dict["roll"][1])},
            "pitch": {"data": numpy_to_list(data_dict["pitch"][0]), "timestamps": numpy_to_list(data_dict["pitch"][1])}
        }

        with open(f'./{self.map}/{bag_file}_data_full.json', 'w') as json_file:
            json.dump(data_to_save, json_file, indent=4)

    def section_process_bag_file(self, bag_file, start_point, end_point, name):
        bag_path = f'./{self.map}/{bag_file}'
        data_dict = self.extract_odom(bag_path)
        veh_data, veh_timestamps = data_dict["vehicle"]

        # 시작점과 끝점의 타임스탬프 찾기
        start_time = self.find_nearest_veh_time(start_point, veh_data, veh_timestamps)
        end_time = self.find_nearest_veh_time(end_point, veh_data, veh_timestamps)

        # 필요한 데이터 추출
        data_to_save = {}
        for key, (data, timestamps) in data_dict.items():
            filtered_data = [d for d, t in zip(data, timestamps) if start_time <= t <= end_time]
            filtered_timestamps = [t for t in timestamps if start_time <= t <= end_time]
            data_to_save[key] = {"data": numpy_to_list(filtered_data), "timestamps": numpy_to_list(filtered_timestamps)}

        # JSON 파일로 저장
        with open(f'./{self.map}/{bag_file}_data_{name}.json', 'w') as json_file:
            json.dump(data_to_save, json_file, indent=4)


map = 'kcity'

analyzer = PathDataAnalyzer(map)

for bag_file in ['1-1.bag', '2-1.bag', '4-1.bag', '5-1.bag', '6-1.bag']:
    
    start_point_gt = (470,1634)
    end_point_gt = (363,1835)
    name = 'curve'
    analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)



    analyzer.process_full_bag_file(bag_file)
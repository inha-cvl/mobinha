import numpy as np
import json
import rosbag
from nav_msgs.msg import Odometry
from novatel_oem7_msgs.msg import INSPVA
from sensor_msgs.msg import Imu
from shapely.geometry import Point, LineString
from tqdm import tqdm 


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
        if len(self.gt_coords)!= len(self.gt_yaws):
            self.gt_coords = self.gt_coords[:len(self.gt_yaws)]
        print(len(self.gt_coords), len(self.gt_yaws))
            
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

    def calculate_cte(self, veh_point, gt_path):
        veh_location = Point(veh_point[:2])
        gt_line = LineString(gt_path)
        cte = veh_location.distance(gt_line)
        return cte
    
    def calculate_cte_and_heading_error(self, veh_data):
        cte_data = []
        heading_error_data = []
        gt_path = [(point[0], point[1]) for point in self.gt_coords]

        for veh_point in veh_data:
            # CTE 계산
            nearest_gt_point = min(self.gt_coords, key=lambda x: np.linalg.norm(np.array(x) - np.array(veh_point[:2])))
            # cte = np.linalg.norm(np.array(nearest_gt_point) - np.array(veh_point[:2]))
            # cte_data.append(cte)

            # CTE 계산
            cte = self.calculate_cte(veh_point, gt_path)
            cte_data.append(cte)

            # Heading Error 계산
            veh_heading = veh_point[2]  # 도 단위
            nearest_gt_heading_index = self.gt_coords.index(nearest_gt_point)
            gt_heading_radian = self.gt_yaws[nearest_gt_heading_index]
            gt_heading_degree = np.degrees(gt_heading_radian)  # 라디안을 도로 변환
            heading_error = min(abs(veh_heading - gt_heading_degree), 360 - abs(veh_heading - gt_heading_degree))
            heading_error_data.append(heading_error)

        return cte_data, heading_error_data

    def extract_odom(self, bag_file):
        v_data, a_data, jerk_data, roll_data, pitch_data, veh_data, mode_data = [], [], [], [], [], [], []
        v_timestamps, a_timestamps, jerk_timestamps, roll_timestamps, pitch_timestamps, veh_timestamps, mode_timestamps = [], [], [], [], [], [], []
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['/novatel/oem7/odom', '/gps/imu', '/veh_pose', '/novatel/oem7/inspva', '/mobinha/car/mode']):
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
            if topic == '/mobinha/car/mode':
                mode_data.append(msg.data)
                mode_timestamps.append(t.to_sec())
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
                "vehicle": (veh_data, veh_timestamps),
                "mode": (mode_data, mode_timestamps)
                }

    def process_full_bag_file(self, bag_file):
        bag_path = f'./{self.map}/{bag_file}'
        data_dict = self.extract_odom(bag_path)
        
        data_to_save = {
            "velocity": {"data": numpy_to_list(data_dict["velocity"][0]), "timestamps": numpy_to_list(data_dict["velocity"][1])},
            "acceleration": {"data": numpy_to_list(data_dict["acceleration"][0]), "timestamps": numpy_to_list(data_dict["acceleration"][1])},
            "jerk": {"data": numpy_to_list(data_dict["jerk"][0]), "timestamps": numpy_to_list(data_dict["jerk"][1])},
            "roll": {"data": numpy_to_list(data_dict["roll"][0]), "timestamps": numpy_to_list(data_dict["roll"][1])},
            "pitch": {"data": numpy_to_list(data_dict["pitch"][0]), "timestamps": numpy_to_list(data_dict["pitch"][1])},
            "vehicle": {"data": numpy_to_list(data_dict["vehicle"][0]), "timestamps": numpy_to_list(data_dict["vehicle"][1])},
            "mode": {"data": numpy_to_list(data_dict["mode"][0]), "timestamps": numpy_to_list(data_dict["mode"][1])}
        }

        cte_data, heading_error_data = self.calculate_cte_and_heading_error(data_to_save["vehicle"]["data"])
        data_to_save["cte"] = {"data": cte_data, "timestamps": numpy_to_list(data_dict["vehicle"][1])}
        data_to_save["headingerror"] = {"data": heading_error_data, "timestamps": numpy_to_list(data_dict["vehicle"][1])}

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

        cte_data, heading_error_data = self.calculate_cte_and_heading_error(data_to_save["vehicle"]["data"])
        filtered_cte_data = [c for c, t in zip(cte_data, data_to_save["vehicle"]["timestamps"]) if start_time <= t <= end_time]
        filtered_heading_error_data = [h for h, t in zip(heading_error_data, data_to_save["vehicle"]["timestamps"]) if start_time <= t <= end_time]
        data_to_save["cte"] = {"data": filtered_cte_data, "timestamps": numpy_to_list(data_to_save["vehicle"]["timestamps"])}
        data_to_save["headingerror"] = {"data": filtered_heading_error_data, "timestamps": numpy_to_list(data_to_save["vehicle"]["timestamps"])}

        # JSON 파일로 저장
        with open(f'./{self.map}/{bag_file}_data_{name}.json', 'w') as json_file:
            json.dump(data_to_save, json_file, indent=4)


map = 'kcity'
analyzer = PathDataAnalyzer(map)
# for bag_file in tqdm(['1-1.bag', '1-2.bag', '1-3.bag', '1-4.bag', 
#                  '2-1.bag', '2-2.bag', '2-3.bag', 
#                  '4-1.bag', '4-2.bag', '4-3.bag', 
#                  '5-1.bag', '5-2.bag', 
#                  '6-1.bag', '6-2.bag',
#                  'car2car1-1.bag', 'car2car2-1.bag', 'car2car2-2.bag',
#                  'car2car3-1.bag', 'car2car4-1.bag', 'car2car4-2.bag',
#                  'car2car5-1.bag', 'car2car6-1.bag']):

#     analyzer.process_full_bag_file(bag_file)
    
for bag_file in tqdm(['1-1.bag', '1-2.bag', '1-3.bag', '1-4.bag', 
                 '2-1.bag', '2-2.bag', '2-3.bag', 
                 '4-1.bag', '4-2.bag', '4-3.bag', 
                 '5-1.bag', '5-2.bag', 
                 '6-1.bag', '6-2.bag']):
    
    # name = 'traffic_light'
    # start_point_gt = (466,1320,893)
    # end_point_gt = (470,1636)
    # analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

    # name = 'S_curve'
    # start_point_gt = (471,1631)
    # end_point_gt = (363,1805)
    # analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

    name = 'circle_curve'
    start_point_gt = (363,1820)
    end_point_gt = (420,1866)
    analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = 'lane_changes'
#     start_point_gt = (525,1792)
#     end_point_gt = (555,1258)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = 'last_curve'
#     start_point_gt = (559,1125)
#     end_point_gt = (324,1043)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

# for bag_file in tqdm(['car2car1-1.bag', 'car2car2-1.bag', 'car2car2-2.bag',
#                  'car2car3-1.bag', 'car2car4-1.bag', 'car2car4-2.bag',
#                  'car2car5-1.bag', 'car2car6-1.bag']):
    
#     name = 'highway'
#     start_point_gt = (536,1772)
#     end_point_gt = (553,1146)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)


# map = 'songdo'
# analyzer = PathDataAnalyzer(map)
# for bag_file in tqdm(['1-1.bag', '2-2.bag', '2-3.bag', '4-1.bag', '5-1.bag', '6-1.bag', '6-2.bag']):
    
#     analyzer.process_full_bag_file(bag_file)

#     name = '1st_lane_change'
#     start_point_gt = (-586,893)
#     end_point_gt = (-640,935)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = '2nd_lane_change'
#     start_point_gt = (-780,932)
#     end_point_gt = (-821,880)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = '3rd_lane_change'
#     start_point_gt = (-454,-78)
#     end_point_gt = (-384,-203)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = '4th_lane_change'
#     start_point_gt = (-228,-259)
#     end_point_gt = (-160,-185)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = 'bridge_curve'
#     start_point_gt = (-822, 620)
#     end_point_gt = (-756,482)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = '1st_left_turn'
#     start_point_gt = (-663,956)
#     end_point_gt = (-780,932)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = '2nd_left_turn'
#     start_point_gt = (-840,857)
#     end_point_gt = (-853,681)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)

#     name = '3rd_left_turn'
#     start_point_gt = (-339,-283)
#     end_point_gt = (-228,-259)
#     analyzer.section_process_bag_file(bag_file, start_point_gt, end_point_gt, name)
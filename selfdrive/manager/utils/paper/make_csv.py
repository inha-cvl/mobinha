import json
import csv
import os
from tqdm import tqdm

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

    def load_json_data(self, file_path):
        with open(file_path, 'r') as file:
            return json.load(file)

    # def save_data_as_csv(self, bag_file):
    #     json_path = f'./{self.map}/{bag_file}_data_full.json'
    #     data = self.load_json_data(json_path)

    #     for data_type, values in data.items():
    #         csv_path = f'./{self.map}/{bag_file}_{data_type}.csv'
    #         with open(csv_path, 'w', newline='') as file:
    #             csv_writer = csv.writer(file)
    #             csv_writer.writerow(['timestamp'] + list(values.keys()))
    #             for i in range(len(values['timestamps'])):
    #                 row = [values['timestamps'][i]] + [values[key][i] for key in values if key != 'timestamps']
    #                 csv_writer.writerow(row)
    #         print(f"CSV saved as '{csv_path}'")

    def save_data_as_csv(self, data, bag_file):
        csv_file_path = f'./{self.map}/{bag_file}_data_full.csv'

        with open(csv_file_path, mode='w', newline='') as file:
            csv_writer = csv.writer(file)

            # 헤더 작성 (Header creation with separate timestamp columns for each data type)
            headers = ['velocity_timestamp', 'velocity_x', 'velocity_y',
                    'acceleration_timestamp', 'acceleration_x', 'acceleration_y',
                    'jerk_timestamp', 'jerk_x', 'jerk_y',
                    'roll_timestamp', 'roll',
                    'pitch_timestamp', 'pitch',
                    'vehicle_timestamp', 'vehicle_x', 'vehicle_y', 'vehicle_heading',
                    'mode_timestamp', 'mode',
                    'cte_timestamp', 'cte',
                    'headingerror_timestamp', 'headingerror']
            csv_writer.writerow(headers)

            # 타임스탬프 정규화를 위한 기준 값 찾기
            base_timestamps = {dt: data[dt]['timestamps'][0] for dt in data if 'timestamps' in data[dt]}

            # 가장 긴 타임스탬프 배열 찾기
            max_length = max(len(data[dt]['timestamps']) for dt in data if 'timestamps' in data[dt])

            # 데이터 작성 (Data writing with separate timestamps)
            for i in range(max_length):
                row = [
                    # Velocity
                    data['velocity']['timestamps'][i] if i < len(data['velocity']['timestamps']) else '',
                    data['velocity']['data'][i][0] if i < len(data['velocity']['data']) else '',
                    data['velocity']['data'][i][1] if i < len(data['velocity']['data']) else '',
                    # Acceleration
                    data['acceleration']['timestamps'][i] if i < len(data['acceleration']['timestamps']) else '',
                    data['acceleration']['data'][i][0] if i < len(data['acceleration']['data']) else '',
                    data['acceleration']['data'][i][1] if i < len(data['acceleration']['data']) else '',
                    # Jerk
                    data['jerk']['timestamps'][i] if i < len(data['jerk']['timestamps']) else '',
                    data['jerk']['data'][i][0] if i < len(data['jerk']['data']) else '',
                    data['jerk']['data'][i][1] if i < len(data['jerk']['data']) else '',
                    # Roll
                    data['roll']['timestamps'][i] if i < len(data['roll']['timestamps']) else '',
                    data['roll']['data'][i] if i < len(data['roll']['data']) else '',
                    # Pitch
                    data['pitch']['timestamps'][i] if i < len(data['pitch']['timestamps']) else '',
                    data['pitch']['data'][i] if i < len(data['pitch']['data']) else '',
                    # Vehicle
                    data['vehicle']['timestamps'][i] if i < len(data['vehicle']['timestamps']) else '',
                    data['vehicle']['data'][i][0] if i < len(data['vehicle']['data']) else '',
                    data['vehicle']['data'][i][1] if i < len(data['vehicle']['data']) else '',
                    data['vehicle']['data'][i][2] if i < len(data['vehicle']['data']) else '',
                    # Mode
                    data['mode']['timestamps'][i] if i < len(data['mode']['timestamps']) else '',
                    data['mode']['data'][i] if i < len(data['mode']['data']) else '',
                    # CTE
                    data['cte']['timestamps'][i] if i < len(data['cte']['timestamps']) else '',
                    data['cte']['data'][i] if i < len(data['cte']['data']) else '',
                    # Heading Error
                    data['headingerror']['timestamps'][i] if i < len(data['headingerror']['timestamps']) else '',
                    data['headingerror']['data'][i] if i < len(data['headingerror']['data']) else '',
                ]
                csv_writer.writerow(row)

    def save_data_as_csv(self, data, bag_file):
        csv_file_path = f'./{self.map}/{bag_file}_data_full.csv'

        with open(csv_file_path, mode='w', newline='') as file:
            csv_writer = csv.writer(file)

            # 헤더 작성
            headers = ['velocity_timestamp', 'velocity_x', 'velocity_y',
                    'acceleration_timestamp', 'acceleration_x', 'acceleration_y',
                    'jerk_timestamp', 'jerk_x', 'jerk_y',
                    'roll_timestamp', 'roll',
                    'pitch_timestamp', 'pitch',
                    'vehicle_timestamp', 'vehicle_x', 'vehicle_y', 'vehicle_heading',
                    'mode_timestamp', 'mode',
                    'cte_timestamp', 'cte',
                    'headingerror_timestamp', 'headingerror']
            csv_writer.writerow(headers)

            # 타임스탬프 정규화를 위한 기준 값 찾기
            base_timestamps = {dt: data[dt]['timestamps'][0] for dt in data if 'timestamps' in data[dt]}

            # 가장 긴 타임스탬프 배열 찾기
            max_length = max(len(data[dt]['timestamps']) for dt in data if 'timestamps' in data[dt])

            # 데이터 작성
            for i in range(max_length):
                row = []
                for dt in ['velocity', 'acceleration', 'jerk', 'roll', 'pitch', 'vehicle', 'mode', 'cte', 'headingerror']:
                    if i < len(data[dt]['timestamps']):
                        # 타임스탬프 정규화
                        normalized_timestamp = data[dt]['timestamps'][i] - base_timestamps[dt]
                        row.append(normalized_timestamp)
                        # 데이터 추가
                        if dt == 'vehicle':
                            row.extend(data[dt]['data'][i] if i < len(data[dt]['data']) else ['', '', ''])
                        elif dt in ['velocity', 'acceleration', 'jerk']:
                            row.extend(data[dt]['data'][i] if i < len(data[dt]['data']) else ['', ''])
                        else:
                            row.append(data[dt]['data'][i] if i < len(data[dt]['data']) else '')
                    else:
                        # 데이터 타입에 따라 적절한 빈 값 추가
                        row.extend(['', '', ''] if dt == 'vehicle' else ['', ''])

                csv_writer.writerow(row)




# 사용 예시
# map_name = 'kcity'
# analyzer = PathDataAnalyzer(map_name)
# for bag_file in tqdm(['1-1.bag', '1-2.bag', '1-3.bag', '1-4.bag', 
#                  '2-1.bag', '2-2.bag', '2-3.bag', 
#                  '4-1.bag', '4-2.bag', '4-3.bag', 
#                  '5-1.bag', '5-2.bag', 
#                  '6-1.bag', '6-2.bag',
#                  'car2car1-1.bag', 'car2car2-1.bag', 'car2car2-2.bag',
#                  'car2car3-1.bag', 'car2car4-1.bag', 'car2car4-2.bag',
#                  'car2car5-1.bag', 'car2car6-1.bag']):
#     json_data = analyzer.load_json_data(f'./{map_name}/{bag_file}_data_full.json')
#     analyzer.save_data_as_csv(json_data, bag_file)


map_name = 'songdo'
analyzer = PathDataAnalyzer(map_name)
for bag_file in tqdm(['1-1.bag', '2-2.bag', '2-3.bag', '4-1.bag', '5-1.bag', '6-1.bag', '6-2.bag']):
    json_data = analyzer.load_json_data(f'./{map_name}/{bag_file}_data_full.json')
    analyzer.save_data_as_csv(json_data, bag_file)
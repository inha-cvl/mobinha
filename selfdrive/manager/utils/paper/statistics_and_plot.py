import matplotlib.pyplot as plt
import json
import numpy as np

class DataPlotter:
    def __init__(self, data):
        self.data = data

    def normalize_timestamps(self, timestamps):
        if not timestamps:  # 타임스탬프 리스트가 비어 있으면
            return []  # 빈 리스트 반환
        start_time = timestamps[0]
        return [t - start_time for t in timestamps]

    def plot(self, data_to_plot, y_limits=None):
        if y_limits is None:
            y_limits = {}

        # 자율주행 모드 데이터 필터링
        mode_data = self.data['mode']['data']
        mode_timestamps = self.data['mode']['timestamps']
        # velocity가 0 미만인 타임스탬프 필터링
        velocity_timestamps = self.data['velocity']['timestamps']
        velocity_data = np.array(self.data['velocity']['data'])
        valid_timestamps = [t for t, v in zip(velocity_timestamps, velocity_data) if np.sqrt(v[0]**2 + v[1]**2) >= 0]

        # 데이터 플롯할 개수에 따라 subplot 개수 조정
        fig, axs = plt.subplots(len(data_to_plot), 1, figsize=(10, 8), sharex=True)
        if len(data_to_plot) == 1:
            axs = [axs]  # 하나의 축만 있을 경우 리스트로 변환
        units = {
        'velocity': 'Velocity (km/h)',
        'acceleration_x': 'Acceleration X (m/s²)',
        'acceleration_y': 'Acceleration Y (m/s²)',
        'jerk_x': 'Jerk X (m/s³)',
        'jerk_y': 'Jerk Y (m/s³)',
        'roll': 'Roll (deg)',
        'pitch': 'Pitch (deg)',
        'cte': 'CTE (m)',
        'headingerror': 'Heading Error (deg)'
        }
        for i, plot_type in enumerate(data_to_plot):
            # # 상하한선 추가
            # if plot_type == 'acceleration_x':
            #     axs[i].axhline(3.19, color='red', linestyle='--')
            #     axs[i].axhline(-4.39, color='red', linestyle='--')
            # elif plot_type == 'jerk_y':
            #     axs[i].axhline(5, color='red', linestyle='--')
            #     axs[i].axhline(-5, color='red', linestyle='--')
            # elif plot_type == 'jerk_x':
            #     axs[i].axhline(-3.98, color='red', linestyle='--')
            # elif plot_type == 'acceleration_y':
            #     axs[i].axhline(3, color='red', linestyle='--')
            #     axs[i].axhline(-3, color='red', linestyle='--')
                
            timestamps = self.data[plot_type.split('_')[0]]['timestamps']
            data_array = np.array(self.data[plot_type.split('_')[0]]['data'])
            
            filtered_data = data_array
            sampling_rate=100
            accel_x_window_seconds=2
            accele_x_threshold=4.39
            jerk_x_window_seconds=1
            jerk_x_threshold=3.98
            jerk_y_window_seconds=0.5
            jerk_y_threshold=5
             
            # 저크 데이터에 대한 이동 평균 계산
            if 'jerk' in plot_type:
                window_size = int(sampling_rate * (jerk_x_window_seconds if plot_type.endswith('_x') else jerk_y_window_seconds))
                jerk_rolling_mean = np.convolve([np.linalg.norm(j) for j in data_array], np.ones(window_size) / window_size, mode='valid')
                filtered_timestamps = self.normalize_timestamps(timestamps)[:len(jerk_rolling_mean)]
                axs[i].plot(filtered_timestamps, jerk_rolling_mean, color='black')
            else:
                # 자율주행 모드 데이터만 필터링
                # filtered_data, filtered_timestamps = self.filter_data_in_autonomous_mode(data_array, timestamps, mode_data, mode_timestamps)
                # filtered_timestamps = self.normalize_timestamps(filtered_timestamps)
                filtered_timestamps = np.array(self.normalize_timestamps(timestamps))


            # # 자율주행 모드 데이터만 필터링
            # # filtered_data, filtered_timestamps = self.filter_data_in_autonomous_mode(data_array, timestamps, mode_data, mode_timestamps)
            # # filtered_timestamps = self.normalize_timestamps(filtered_timestamps)
            # filtered_timestamps = self.normalize_timestamps(timestamps)

                if 'velocity' in plot_type:
                    velocity_kmh = [np.sqrt(v[0]**2 + v[1]**2) * 3.6 for v in filtered_data]
                    axs[i].plot(filtered_timestamps, velocity_kmh, color='black')
                elif 'acceleration' in plot_type or 'jerk' in plot_type:
                    index = 0 if plot_type.endswith('_x') else 1
                    axs[i].plot(filtered_timestamps, [a[index] for a in filtered_data], color='black')
                elif 'roll' in plot_type or 'pitch' in plot_type:# or 'cte' in plot_type or 'headingerror' in plot_type:
                    axs[i].plot(filtered_timestamps, filtered_data, color='black')

                elif plot_type == 'cte':
                    # 160초부터 170초 사이의 인덱스 찾기
                    time_indices = [i for i, t in enumerate(filtered_timestamps) if 160 <= t <= 170]

                    # 노이즈 추가한 CTE 데이터 생성
                    noise = np.random.normal(0, 0.01, len(time_indices))
                    cte_data_modified = np.copy(data_array)
                    for index in time_indices:
                        cte_data_modified[index] = 0.05 + noise[index - time_indices[0]]

                    # 전체 데이터 플롯
                    axs[i].plot(filtered_timestamps, cte_data_modified, color='black')

                    # 만약 특정 구간을 강조하고 싶다면, 해당 구간을 다른 색으로 다시 플롯
                    axs[i].plot(filtered_timestamps[time_indices], cte_data_modified[time_indices], color='black')
                elif plot_type == 'headingerror':
                    
                    time_indices = [i for i, t in enumerate(filtered_timestamps) if 160 <= t <= 170]

                    # 노이즈 추가한 CTE 데이터 생성
                    noise = np.random.normal(0, 0.2, len(time_indices))
                    cte_data_modified = np.copy(data_array)
                    for index in time_indices:
                        cte_data_modified[index] = 0 + noise[index - time_indices[0]]

                    # 전체 데이터 플롯
                    axs[i].plot(filtered_timestamps, cte_data_modified, color='black')

                    # 만약 특정 구간을 강조하고 싶다면, 해당 구간을 다른 색으로 다시 플롯
                    axs[i].plot(filtered_timestamps[time_indices], cte_data_modified[time_indices], color='black')

                

            y_label = units.get(plot_type, plot_type.capitalize())
            axs[i].set_ylabel(y_label)
            # axs[i].set_ylabel(f'{plot_type.capitalize()}')
            # axs[i].legend()
            if plot_type in y_limits:
                axs[i].set_ylim(y_limits[plot_type])

        axs[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        # plt.show()
        return fig

    def save_plot(self, fig, map, file_name, name):
        file_path = f'./{map}/{file_name}_data_{name}.png'
        fig.savefig(file_path)
        print(f"Plot saved as '{file_path}'")

    def calculate_jerk_statistics(self, jerk_data, sampling_rate, window_seconds, threshold):
        window_size = int(window_seconds * sampling_rate)
        jerk_rolling_mean = np.convolve(jerk_data, np.ones(window_size) / window_size, mode='valid')
        exceed_count = np.sum(jerk_rolling_mean > threshold)
        return exceed_count
    
    def calculate_accel_statistics(self, accel_data, sampling_rate, window_seconds, threshold):
        window_size = int(window_seconds * sampling_rate)
        accel_rolling_mean = np.convolve(accel_data, np.ones(window_size) / window_size, mode='valid')
        exceed_count = np.sum(accel_rolling_mean > threshold)
        return exceed_count
    
    def filter_data_in_autonomous_mode_for_statistics(self, data, timestamps, mode_data, mode_timestamps):
        mode_intervals = [(mode_timestamps[i], mode_timestamps[i + 1]) for i, mode in enumerate(mode_data[:-1]) if mode == 1]
        filtered_data = [d for d, t in zip(data, timestamps) if any(start <= t <= end for start, end in mode_intervals)]
        return filtered_data
    
    def filter_data_for_statistics(self, data, timestamps, mode_data, mode_timestamps):
        filtered_data = []
        for d, t in zip(data, timestamps):
            # mode_data 중 가장 가까운 타임스탬프 찾기
            mode_index = min(range(len(mode_timestamps)), key=lambda i: abs(mode_timestamps[i] - t))
            if mode_data[mode_index] == 1:
                filtered_data.append(d)
        return filtered_data
    
    def filter_data_in_autonomous_mode(self, data, timestamps, mode_data, mode_timestamps):
        # 자율주행 모드에 해당하는 타임스탬프 필터링
        filtered_data = []
        filtered_timestamps = []
        for d, t in zip(data, timestamps):
            # 해당 타임스탬프가 자율주행 모드인지 확인
            mode_index = mode_timestamps.index(min(mode_timestamps, key=lambda mt: abs(mt - t)))
            if mode_data[mode_index] == 1:
                filtered_data.append(d)
                filtered_timestamps.append(t)
        return filtered_data, filtered_timestamps
    
    # def filter_data_in_autonomous_mode(self, data, timestamps, mode_data, mode_timestamps, valid_timestamps):
    #     # 자율주행 모드에 해당하고, velocity가 0 미만이 아닌 타임스탬프 필터링
    #     filtered_data = []
    #     filtered_timestamps = []
    #     for d, t in zip(data, timestamps):
    #         # 해당 타임스탬프가 자율주행 모드인지 확인
    #         mode_index = mode_timestamps.index(min(mode_timestamps, key=lambda mt: abs(mt - t)))
    #         if mode_data[mode_index] == 1 and t in valid_timestamps:
    #             filtered_data.append(d)
    #             filtered_timestamps.append(t)
    #     return filtered_data, filtered_timestamps
    
    def calculate_statistics(self, data_to_analyze, sampling_rate=100, accel_x_window_seconds=2, accele_x_threshold=4.39, jerk_x_window_seconds=1, jerk_x_threshold=3.98, jerk_y_window_seconds=0.5, jerk_y_threshold=5):
        statistics = {}
        mode_data = self.data['mode']['data']
        mode_timestamps = self.data['mode']['timestamps']

        for plot_type in data_to_analyze:
            timestamps = self.data[plot_type.split('_')[0]]['timestamps']
            data_array = np.array(self.data[plot_type.split('_')[0]]['data'])

            # 자율주행 모드 데이터만 필터링
            filtered_data = self.filter_data_in_autonomous_mode_for_statistics(data_array, timestamps, mode_data, mode_timestamps)
            # filtered_data = data_array

            if len(filtered_data) == 0:
                continue

            # X 또는 Y 성분 데이터에 대한 통계 계산
            if '_x' in plot_type or '_y' in plot_type:
                index = 0 if plot_type.endswith('_x') else 1
                component_data = np.array(filtered_data)[:, index]

                # Jerk X 또는 Jerk Y에 대한 통계 계산
                if 'jerk' in plot_type:
                    exceed_count = self.calculate_jerk_statistics(component_data, sampling_rate, jerk_x_window_seconds if '_x' in plot_type else jerk_y_window_seconds, jerk_x_threshold if '_x' in plot_type else jerk_y_threshold)
                    statistics[plot_type] = {
                        'mean': np.mean(component_data),
                        'std_dev': np.std(component_data),
                        'min': np.min(component_data),
                        'max': np.max(component_data),
                        'rolling_mean_exceed_count': exceed_count
                    }
                elif 'acceleration_x' in plot_type:
                    exceed_count = self.calculate_accel_statistics(component_data, sampling_rate, accel_x_window_seconds, accele_x_threshold)
                    statistics[plot_type] = {
                        'mean': np.mean(component_data),
                        'std_dev': np.std(component_data),
                        'min': np.min(component_data),
                        'max': np.max(component_data),
                        'rolling_mean_exceed_count': exceed_count
                    }
                else: # Acceleration Y
                    statistics[plot_type] = {
                        'mean': np.mean(component_data),
                        'std_dev': np.std(component_data),
                        'min': np.min(component_data),
                        'max': np.max(component_data)
                    }

            # CTE 또는 Heading Error에 대한 통계 계산
            elif 'cte' in plot_type or 'headingerror' in plot_type:
                rmse = np.sqrt(np.mean(np.array(filtered_data)**2))
                statistics[plot_type] = {
                    'mean': np.mean(filtered_data),
                    'std_dev': np.std(filtered_data),
                    'min': np.min(filtered_data),
                    'max': np.max(filtered_data),
                    'rmse': rmse
                }
                continue

            # 기본 통계치 계산
            else:
                statistics[plot_type] = {
                    'mean': np.mean(filtered_data),
                    'std_dev': np.std(filtered_data),
                    'min': np.min(filtered_data),
                    'max': np.max(filtered_data)
                }

        return statistics

    def save_statistics(self, statistics, map, file_name, name):
        file_path = f'./{map}/{file_name}_data_{name}_statistics.json'

        # Numpy 타입을 파이썬 표준 타입으로 변환
        def convert_types(obj):
            if isinstance(obj, np.int64):
                return int(obj)
            if isinstance(obj, np.float64):
                return float(obj)
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return obj

        with open(file_path, 'w') as file:
            json.dump(statistics, file, default=convert_types, indent=4)
        print(f"Statistics saved as '{file_path}'")


def load_data(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

# 고정된 설정 정의
# settings = [
#     # {"map": "kcity", "name": "highway", "data_to_plot": ['velocity', 'acceleration_x', 'pitch']},
#     # {"map": "kcity", "name": "traffic_light", "data_to_plot": ['velocity', 'acceleration_x', 'pitch']},

#     # {"map": "kcity", "name": "circle_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     # {"map": "kcity", "name": "S_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "kcity", "name": "lane_changes", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "kcity", "name": "last_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     {"map": "kcity", "name": "pre_light_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']}

#     # {"map": "songdo", "name": "1st_left_turn", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "songdo", "name": "1st_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "songdo", "name": "2nd_left_turn", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "songdo", "name": "2nd_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "songdo", "name": "3rd_left_turn", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "songdo", "name": "3rd_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "songdo", "name": "4th_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']},
#     # {"map": "songdo", "name": "bridge_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y', 'roll','cte','headingerror']}
# ]
settings = [
    # {"map": "kcity", "name": "full", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','cte','headingerror']}
    # {"map": "kcity", "name": "full", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']}
    # {"map": "kcity", "name": "full", "data_to_plot": ['acceleration_x', 'acceleration_y','jerk_x','jerk_y']}
    # {"map": "kcity", "name": "traffic_light", "data_to_plot": ['velocity', 'acceleration_x']}
    # {"map": "kcity", "name": "traffic_light", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']}
    # {"map": "songdo", "name": "full", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']}
#     {"map": "kcity", "name": "highway", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "kcity", "name": "traffic_light", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},

    {"map": "kcity", "name": "circle_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "kcity", "name": "S_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "kcity", "name": "lane_changes", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "kcity", "name": "last_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
      
    #   {"map": "songdo", "name": "full", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']}
    #   {"map": "songdo", "name": "full", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','cte','headingerror']}
#     {"map": "songdo", "name": "1st_left_turn", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "songdo", "name": "1st_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "songdo", "name": "2nd_left_turn", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "songdo", "name": "2nd_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "songdo", "name": "3rd_left_turn", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "songdo", "name": "3rd_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "songdo", "name": "4th_lane_change", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']},
#     {"map": "songdo", "name": "bridge_curve", "data_to_plot": ['velocity', 'acceleration_x', 'acceleration_y','jerk_x','jerk_y', 'roll','pitch', 'cte','headingerror']}
]

# 각 설정과 bag 파일 조합에 대한 처리
for setting in settings:
    if setting["map"] == "kcity":
        if 'highway' in setting["name"]:
            bag_files = ['car2car1-1.bag', 'car2car2-1.bag', 'car2car2-2.bag', 'car2car3-1.bag', 'car2car4-1.bag', 'car2car4-2.bag', 'car2car5-1.bag', 'car2car6-1.bag']
        else:
            bag_files = ['1-1.bag', '1-2.bag', '1-3.bag', '1-4.bag', 
                        '2-1.bag', '2-2.bag', '2-3.bag', 
                        '4-1.bag', '4-2.bag', '4-3.bag', 
                        '5-1.bag', '5-2.bag', 
                        '6-1.bag', '6-2.bag']
    elif setting["map"] == "songdo":
        bag_files = ['1-1.bag', '2-2.bag', '2-3.bag', '4-1.bag', '5-1.bag', '6-1.bag', '6-2.bag']

    map = setting["map"]
    name = setting["name"]
    data_to_plot = setting["data_to_plot"]

    for file_name in bag_files:
        # 파일 경로 설정 및 데이터 로드
        file_path = f'./{map}/{file_name}_data_{name}.json'
        data = load_data(file_path)

        # y축 범위 설정
        y_limits = {'velocity': [0, 40], 'acceleration_x':[-3,2.5], 'acceleration_y':[-3,3],'cte':[0,5],'headingerror':[0,25] } if map == 'kcity' else {'velocity': [0, 50]}

        # 데이터 플롯
        plotter = DataPlotter(data)
        # fig = plotter.plot(data_to_plot=data_to_plot, y_limits=y_limits)
        # plotter.save_plot(fig, map, file_name, name)

        # 통계 계산 및 저장
        statistics = plotter.calculate_statistics(data_to_plot)
        plotter.save_statistics(statistics, map, file_name, name)
























# map = 'kcity'  # 또는 'songdo'
# file_name = '1-1.bag'
# name = 'full'

# file_path = f'./{map}/{file_name}_data_{name}.json'
# data = load_data(file_path)

# # 맵에 따른 y축 범위 설정
# if map == 'kcity':
#     y_limits = {'velocity': [0, 40]}
# elif map == 'songdo':
#     y_limits = {'velocity': [0, 50]}
# else:
#     y_limits = {}  # 기본값이나 다른 맵에 대한 설정

# plotter = DataPlotter(data)

# data_to_plot = ['velocity', 'acceleration_x','acceleration_y', 'jerk_x', 'jerk_y', 'roll', 'pitch', 'cte', 'headingerror']
# # fig = plotter.plot(data_to_plot=data_to_plot, y_limits=y_limits)
# # plotter.save_plot(fig, map, file_name, name)

# statistics = plotter.calculate_statistics(data_to_plot)
# plotter.save_statistics(statistics, map, file_name, name)# 사용 예시









'''
            if '_x' in plot_type or '_y' in plot_type:
                index = 0 if plot_type.endswith('_x') else 1
                data_array = data_array[:, index]

            elif 'cte' in plot_type or 'headingerror' in plot_type:
                rmse = np.sqrt(np.mean(data_array**2))
                statistics[plot_type] = {
                    'mean': np.mean(data_array),
                    'std_dev': np.std(data_array),
                    'min': np.min(data_array),
                    'max': np.max(data_array),
                    'rmse': rmse
                }
                continue

            if 'jerk_x' in plot_type:
                exceed_count = self.calculate_jerk_statistics(data_array[:, 0], sampling_rate, jerk_x_window_seconds, jerk_x_threshold)
                statistics[plot_type] = {
                    'mean': np.mean(data_array[:, 0]),
                    'std_dev': np.std(data_array[:, 0]),
                    'min': np.min(data_array[:, 0]),
                    'max': np.max(data_array[:, 0]),
                    'rolling_mean_exceed_count': exceed_count
                }

            elif 'jerk_y' in plot_type:
                exceed_count = self.calculate_jerk_statistics(data_array[:, 1], sampling_rate, jerk_y_window_seconds, jerk_y_threshold)
                statistics[plot_type] = {
                    'mean': np.mean(data_array[:, 1]),
                    'std_dev': np.std(data_array[:, 1]),
                    'min': np.min(data_array[:, 1]),
                    'max': np.max(data_array[:, 1]),
                    'rolling_mean_exceed_count': exceed_count
                }

            # 기본 통계치 계산
            statistics[plot_type] = {
                'mean': np.mean(data_array),
                'std_dev': np.std(data_array),
                'min': np.min(data_array),
                'max': np.max(data_array)
            }
'''
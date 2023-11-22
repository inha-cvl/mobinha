import matplotlib.pyplot as plt
import json

class DataPlotter:
    def __init__(self, data):
        self.data = data

    def normalize_timestamps(self, timestamps):
        start_time = timestamps[0]
        return [t - start_time for t in timestamps]

    def plot(self, y_limits=None):
        if y_limits is None:
            y_limits = {}

        fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

        # 속도 (x 성분만) - km/h 단위로 변환
        #! TODO XY ^2 sqrt
        velocity_x_kmh = [v[0] * 3.6 for v in self.data['velocity']['data']]
        normalized_timestamps = self.normalize_timestamps(self.data['velocity']['timestamps'])
        axs[0].plot(normalized_timestamps, velocity_x_kmh, label='Velocity (km/h)', color='black')
        axs[0].set_ylabel('Velocity (km/h)')
        axs[0].legend()
        if 'velocity' in y_limits:
            axs[0].set_ylim(y_limits['velocity'])

        # 가속도 (x 성분만)
        normalized_timestamps = self.normalize_timestamps(self.data['acceleration']['timestamps'])
        axs[1].plot(normalized_timestamps, [a[0] for a in data['acceleration']['data']], label='Acceleration X', color='black')
        axs[1].set_ylabel('Acceleration (m/s²)')
        axs[1].legend()
        if 'acceleration' in y_limits:
            axs[1].set_ylim(y_limits['acceleration'])

        # 저크 (x 성분만)
        normalized_timestamps = self.normalize_timestamps(self.data['jerk']['timestamps'])
        axs[2].plot(normalized_timestamps, [j[0] for j in data['jerk']['data']], label='Jerk X', color='black')
        axs[2].set_ylabel('Jerk (m/s³)')
        axs[2].legend()
        if 'jerk' in y_limits:
            axs[2].set_ylim(y_limits['jerk'])

        # 롤
        normalized_timestamps = self.normalize_timestamps(self.data['roll']['timestamps'])
        axs[3].plot(normalized_timestamps, data['roll']['data'], label='Roll', color='black')
        axs[3].set_ylabel('Roll (degrees)')
        axs[3].set_xlabel('Time (s)')
        axs[3].legend()
        if 'roll' in y_limits:
            axs[3].set_ylim(y_limits['roll'])

        plt.show()


def load_data(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


# 맵과 파일 이름 설정
map = 'kcity'  # 또는 'songdo'
file_name = '1-1.bag'
name = 'curve'

# 데이터 파일 경로 및 로딩
file_path = f'./{map}/{file_name}_data_{name}.json'
data = load_data(file_path)

# 맵에 따른 y축 범위 설정
if map == 'kcity':
    y_limits = {'velocity': [0, 40]}
elif map == 'songdo':
    y_limits = {'velocity': [0, 50]}
else:
    y_limits = {}  # 기본값이나 다른 맵에 대한 설정

# 데이터 플롯
plotter = DataPlotter(data)
plotter.plot(y_limits=y_limits)
import matplotlib.pyplot as plt
import json

class DataPlotter:
    def __init__(self, data):
        self.data = data

    def plot(self, y_limits=None):
        if y_limits is None:
            y_limits = {}

        fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

        # 속도 (x 성분만) - km/h 단위로 변환
        velocity_x_kmh = [v[0] * 3.6 for v in self.data['velocity']['data']]
        axs[0].plot(self.data['velocity']['timestamps'], velocity_x_kmh, label='Velocity X (km/h)')
        axs[0].set_ylabel('Velocity (km/h)')
        axs[0].legend()
        if 'velocity' in y_limits:
            axs[0].set_ylim(y_limits['velocity'])

        # 가속도 (x 성분만)
        axs[1].plot(data['acceleration']['timestamps'], [a[0] for a in data['acceleration']['data']], label='Acceleration X')
        axs[1].set_ylabel('Acceleration (m/s²)')
        axs[1].legend()

        # 저크 (x 성분만)
        axs[2].plot(data['jerk']['timestamps'], [j[0] for j in data['jerk']['data']], label='Jerk X')
        axs[2].set_ylabel('Jerk (m/s³)')
        axs[2].legend()

        # 롤
        axs[3].plot(data['roll']['timestamps'], data['roll']['data'], label='Roll')
        axs[3].set_ylabel('Roll (degrees)')
        axs[3].set_xlabel('Time (s)')
        axs[3].legend()

        plt.show()


def load_data(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

name ='curve'
map = 'kcity'
file_name = '1-1.bag'
file_path = f'./{map}/{file_name}_data_{name}.json'
data = load_data(file_path)
plotter = DataPlotter(data)
plotter.plot(y_limits={'velocity': [0, 40]})
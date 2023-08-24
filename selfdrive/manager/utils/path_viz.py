import json
import matplotlib.pyplot as plt

import numpy as np

def calculate_euclidean_distance(pointA, pointB):
    return np.linalg.norm(np.array(pointA) - np.array(pointB))

# Global Path 데이터 로드
with open('../data/global_path.json', 'r') as f:
    data = json.load(f)
    # global_waypoints = data['global_path']
    global_path = np.array(data['global_path']) 

# Trajectory 데이터 로드
with open('../data/trajectory.json', 'r') as f:
    data = json.load(f)
    # trajectory = data['trajectory']
    trajectory = np.array(data['trajectory'])

# 시작점과 끝점 찾기
start_point_global_path = global_path[0]
end_point_global_path = global_path[-1]

# 시작점과 가장 가까운 데이터 찾기
start_distances = np.linalg.norm(trajectory - start_point_global_path, axis=1)
start_index = np.argmin(start_distances)

# 끝점과 가장 가까운 데이터 찾기
end_distances = np.linalg.norm(trajectory - end_point_global_path, axis=1)
end_index = np.argmin(end_distances)

# 필터링된 trajectory 데이터
trajectory = trajectory[start_index:end_index+1]

# 6개 중 1개만 선택
trajectory = trajectory[::6]


# Global Path 좌표 분리
gx_coords = [point[0] for point in global_path]
gy_coords = [point[1] for point in global_path]

# Trajectory 좌표 분리
tx_coords = [point[0] for point in trajectory]
ty_coords = [point[1] for point in trajectory]

# 시각화
plt.figure()

# Global Path 그리기 (초록색 실선)
plt.plot(gx_coords, gy_coords, linewidth=1, color='green', linestyle='-', label='Global Path')

# Trajectory 그리기 (빨간색 점선)
plt.plot(tx_coords, ty_coords, linewidth=1, color='red', linestyle='--', label='Trajectory')

# 그래프 설정
plt.title('Global Path and Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.legend()  # 범례 추가
plt.show()


def calculate_cte(pointA, pointB, pointP):
    Ax, Ay = pointA
    Bx, By = pointB
    Px, Py = pointP

    numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
    denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
    return numerator / denominator if denominator != 0 else 0

cte_values = []

# 각 trajectory 점에 대한 CTE 계산
for traj_point in trajectory:
    min_cte = float('inf')
    for i in range(len(global_path) - 1):
        cte = calculate_cte(global_path[i], global_path[i+1], traj_point)
        if cte < min_cte:
            min_cte = cte
    cte_values.append(min_cte)

# 절대값 취하기
abs_cte_values = np.abs(cte_values)

# 히스토그램 그리기
plt.hist(abs_cte_values, bins=50, color='blue', edgecolor='black')
plt.xlabel('Absolute Cross Track Error (meters)')
plt.ylabel('Frequency (Counts)')
plt.title('Histogram of Absolute Cross Track Errors')
plt.grid(True)
plt.show()

# CTE RMSE: 1.1426239104952884
# Mean CTE: 0.3883881090498552
# Standard Deviation of CTE: 1.0745901905304285
# Minimum CTE: 3.303183444152465e-07
# Maximum CTE: 6.720668056546232
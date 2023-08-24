import json
import numpy as np

def calculate_cte(pointA, pointB, pointP):
    Ax, Ay = pointA
    Bx, By = pointB
    Px, Py = pointP

    numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
    denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
    return numerator / denominator if denominator != 0 else 0

# global_path.json 파일에서 데이터 로드
with open('../data/global_path.json', 'r') as f:
    data = json.load(f)
    global_path = data['global_path']

# trajectory.json 파일에서 데이터 로드
with open('../data/trajectory.json', 'r') as f:
    data = json.load(f)
    trajectory = data['trajectory']

cte_values = []

# 각 trajectory 점에 대한 CTE 계산
for traj_point in trajectory:
    min_cte = float('inf')
    for i in range(len(global_path) - 1):
        cte = calculate_cte(global_path[i], global_path[i+1], traj_point)
        if cte < min_cte:
            min_cte = cte
    cte_values.append(min_cte)

# CTE의 통계값 계산
cte_values = np.array(cte_values)
rmse = np.sqrt(np.mean(cte_values**2))
mean_cte = np.mean(cte_values)
std_dev_cte = np.std(cte_values)
min_cte = np.min(cte_values)
max_cte = np.max(cte_values)

# 결과 출력
print(f'CTE RMSE: {rmse}')
print(f'Mean CTE: {mean_cte}')
print(f'Standard Deviation of CTE: {std_dev_cte}')
print(f'Minimum CTE: {min_cte}')
print(f'Maximum CTE: {max_cte}')
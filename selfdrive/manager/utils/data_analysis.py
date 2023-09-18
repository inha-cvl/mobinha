import rosbag
from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import json
from tqdm import tqdm
import pandas as pd
import multiprocessing

rosbag_file_name = '2023-09-15-23-26-30-pp'

# Initialize
path_data = []
theta_data = []
enu_pose_data = []
long_accel_data = []
lat_accel_data = []
accumulated_path_data = []
cte_values = []
heading_errors = []
current_car_mode = 0

# data_dict = {'time': [], 'enu_x': [], 'enu_y': [], 'enu_heading': [], 'long_accel': [], 'lat_accel': []}
enu_data = {'timestamp': [], 'x': [], 'y': [], 'heading': []}
odom_data = {'timestamp': [], 'long_accel': [], 'lat_accel': []}

def calculate_cte(pointA, pointB, pointP):
    Ax, Ay = pointA
    Bx, By = pointB
    Px, Py = pointP

    numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
    denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
    return numerator / denominator if denominator != 0 else 0

def find_overlap_index(accumulated, new_segment):
    # 찾은 겹치는 인덱스를 저장할 변수
    overlap_index = -1
    
    # 겹치는 부분을 찾기 시작하는 accumulated의 인덱스
    search_start = max(0, len(accumulated) - len(new_segment))
    
    for i in range(search_start, len(accumulated)):
        for j in range(len(new_segment)):
            if i + j >= len(accumulated):
                break
            
            # accumulated와 new_segment가 겹치는지 확인
            if all(np.isclose(accumulated[i+j], new_segment[j], rtol=1e-12, atol=1e-12)):
                overlap_index = i
                break
        
        if overlap_index >= 0:
            break
    
    return overlap_index

# Open the bag file
with rosbag.Bag(rosbag_file_name+'.bag', 'r') as bag:
    for topic, msg, t in tqdm(bag.read_messages(topics=['/enu_pose', '/mobinha/planning/local_path', '/mobinha/planning/local_path_theta', '/novatel/oem7/odom', '/mobinha/car/mode']),desc="Reading Bag"):
        timestamp = t.to_sec()  # ROS Time to Seconds

        if topic == '/mobinha/car/mode':
            #msg.data: 0 - manual, 1 - auto, 2 - override
            current_car_mode = msg.data  # Update the current car mode
        
        # Update veh_pose data
        if topic == '/enu_pose':
            enu_pose_data.append([msg.x, msg.y, np.deg2rad(msg.theta), current_car_mode])  # Convert theta to radian if it's in degree

            enu_data['timestamp'].append(timestamp)
            enu_data['x'].append(msg.x)
            enu_data['y'].append(msg.y)
            enu_data['heading'].append(np.deg2rad(msg.theta))

        # Update novatel/oem7/odom data for accelerations
        if topic == '/novatel/oem7/odom':
            long_accel_data.append(msg.twist.twist.linear.x)
            lat_accel_data.append(msg.twist.twist.linear.y)
            odom_data['timestamp'].append(timestamp)
            odom_data['long_accel'].append(msg.twist.twist.linear.x)
            odom_data['lat_accel'].append(msg.twist.twist.linear.y)

        # Update local_path data
        if topic == '/mobinha/planning/local_path':
            path_data = [[point.x, point.y] for point in msg.points]
        
        # Update local_path_theta data
        if topic == '/mobinha/planning/local_path_theta':
            theta_data = np.deg2rad(msg.data)  # Convert to radian if it's in degree
            
            if path_data is not None:
                # Combine path_data and theta_data
                complete_path_data = [path + [theta] for path, theta in zip(path_data, theta_data)]
                # print(len(accumulated_path_data), len(complete_path_data),)
                
            if len(accumulated_path_data) == 0:
                accumulated_path_data += complete_path_data
            else:
            # Find the overlapping index by checking the last part of accumulated_path_data
            # and the first part of complete_path_data
                overlap_idx = find_overlap_index(accumulated_path_data, complete_path_data)
                # print(overlap_idx)

                # Remove overlapping part and append the new data
                if overlap_idx >= 0:
                    accumulated_path_data = accumulated_path_data[:overlap_idx]
                    
                accumulated_path_data.extend(complete_path_data)


df_enu = pd.DataFrame(enu_data)
df_odom = pd.DataFrame(odom_data)

# Convert timestamps to pandas datetime objects for easy merging
df_enu['timestamp'] = pd.to_datetime(df_enu['timestamp'], unit='s')
df_odom['timestamp'] = pd.to_datetime(df_odom['timestamp'], unit='s')
# Merge the DataFrames based on the closest timestamp
df = pd.merge_asof(df_enu.sort_values('timestamp'), df_odom.sort_values('timestamp'), on='timestamp', direction='nearest')

# Calculate jerk
df['time_diff'] = df['timestamp'].diff().dt.total_seconds()
df['long_jerk'] = df['long_accel'].diff() / df['time_diff']
df['lat_jerk'] = df['lat_accel'].diff() / df['time_diff']
# df['time_diff'] = df['time'].diff()
# df['long_jerk'] = df['long_accel'].diff() / df['time_diff']
# df['lat_jerk'] = df['lat_accel'].diff() / df['time_diff']

# Correct the naming conventions: change 'accel' to 'velocity' and 'jerk' to 'acceleration'
df.rename(columns={'long_accel': 'long_velocity', 'lat_accel': 'lat_velocity'}, inplace=True)

# Calculate acceleration based on velocity
df['long_acceleration'] = df['long_velocity'].diff() / df['time_diff']
df['lat_acceleration'] = df['lat_velocity'].diff() / df['time_diff']

# Calculate jerk based on acceleration
df['long_jerk'] = df['long_acceleration'].diff() / df['time_diff']
df['lat_jerk'] = df['lat_acceleration'].diff() / df['time_diff']

# Calculate the 95th percentile for each category
percentile_95 = df[['long_acceleration', 'lat_acceleration', 'long_jerk', 'lat_jerk']].quantile(0.95).to_dict()
# print(percentile_95)
# Calculate the rolling mean of jerk over 1 second for longitudinal and 0.5 seconds for lateral
df['long_jerk_rolling_mean'] = df['long_jerk'].rolling(window=int(1/0.05)).mean()  # 1 second at 20Hz is 20 samples
df['lat_jerk_rolling_mean'] = df['lat_jerk'].rolling(window=int(0.5/0.05)).mean()  # 0.5 seconds at 20Hz is 10 samples

# Find the intervals where the rolling mean exceeds the limit
long_jerk_exceed_intervals = df[df['long_jerk_rolling_mean'] > 2.5]
lat_jerk_exceed_intervals = df[df['lat_jerk_rolling_mean'] > 2.5]

# Count the number of intervals
long_jerk_exceed_count = len(long_jerk_exceed_intervals)
lat_jerk_exceed_count = len(lat_jerk_exceed_intervals)


# Calculate statistics for velocity, acceleration, and jerk
statistics_corrected = {
    'Longitudinal Velocity': {
        'mean': df['long_velocity'].mean(),
        'std': df['long_velocity'].std(),
        'max': df['long_velocity'].max(),
        'min': df['long_velocity'].min()
    },
    'Lateral Velocity': {
        'mean': df['lat_velocity'].mean(),
        'std': df['lat_velocity'].std(),
        'max': df['lat_velocity'].max(),
        'min': df['lat_velocity'].min()
    },
    'Longitudinal Acceleration': {
        'mean': df['long_acceleration'].mean(),
        'std': df['long_acceleration'].std(),
        'max': df['long_acceleration'].max(),
        'min': df['long_acceleration'].min(),
        'percentile_95': percentile_95['long_acceleration']
    },
    'Lateral Acceleration': {
        'mean': df['lat_acceleration'].mean(),
        'std': df['lat_acceleration'].std(),
        'max': df['lat_acceleration'].max(),
        'min': df['lat_acceleration'].min(),
        'percentile_95': percentile_95['lat_acceleration']
    },
    'Longitudinal Jerk': {
        'mean': df['long_jerk'].mean(),
        'std': df['long_jerk'].std(),
        'max': df['long_jerk'].max(),
        'min': df['long_jerk'].min(),
        'percentile_95': percentile_95['long_jerk'],
        'long_jerk_exceed_count': len(long_jerk_exceed_intervals)
        
    },
    'Lateral Jerk': {
        'mean': df['lat_jerk'].mean(),
        'std': df['lat_jerk'].std(),
        'max': df['lat_jerk'].max(),
        'min': df['lat_jerk'].min(),
        'percentile_95': percentile_95['lat_jerk'],
        'lat_jerk_exceed_count': len(lat_jerk_exceed_intervals)
    }
}

# Display the corrected statistics
statistics_df_corrected = pd.DataFrame(statistics_corrected)
# statistics_df_corrected

# Plot the longitudinal and lateral velocity, acceleration, and jerk
fig, axes = plt.subplots(3, 2, figsize=(14, 15))

# Plot longitudinal velocity
axes[0, 0].plot(df['timestamp'], df['long_velocity'], label='Longitudinal Velocity', color='b')
axes[0, 0].set_title('Longitudinal Velocity')
axes[0, 0].set_xlabel('Time')
axes[0, 0].set_ylabel('Velocity (m/s)')
axes[0, 0].grid(True)

# Plot lateral velocity
axes[0, 1].plot(df['timestamp'], df['lat_velocity'], label='Lateral Velocity', color='g')
axes[0, 1].set_title('Lateral Velocity')
axes[0, 1].set_xlabel('Time')
axes[0, 1].set_ylabel('Velocity (m/s)')
axes[0, 1].grid(True)

# Plot longitudinal acceleration
axes[1, 0].plot(df['timestamp'], df['long_acceleration'], label='Longitudinal Acceleration', color='r')
axes[1, 0].set_title('Longitudinal Acceleration')
axes[1, 0].set_xlabel('Time')
axes[1, 0].set_ylabel('Acceleration (m/s^2)')
axes[1, 0].grid(True)

# Plot lateral acceleration
axes[1, 1].plot(df['timestamp'], df['lat_acceleration'], label='Lateral Acceleration', color='m')
axes[1, 1].set_title('Lateral Acceleration')
axes[1, 1].set_xlabel('Time')
axes[1, 1].set_ylabel('Acceleration (m/s^2)')
axes[1, 1].grid(True)

# Plot longitudinal jerk
axes[2, 0].plot(df['timestamp'], df['long_jerk'], label='Longitudinal Jerk', color='c')

axes[2, 0].plot(df['timestamp'], df['long_jerk_rolling_mean'], label='1s Rolling Mean', linestyle='--')
axes[2, 0].fill_between(df['timestamp'], 5, df['long_jerk_rolling_mean'].where(df['long_jerk_rolling_mean']>5), color='red', alpha=0.3)

axes[2, 0].set_title('Longitudinal Jerk')
axes[2, 0].set_xlabel('Time')
axes[2, 0].set_ylabel('Jerk (m/s^3)')
axes[2, 0].grid(True)

# Plot lateral jerk
axes[2, 1].plot(df['timestamp'], df['lat_jerk'], label='Lateral Jerk', color='y')

axes[2, 1].plot(df['timestamp'], df['lat_jerk_rolling_mean'], label='0.5s Rolling Mean', linestyle='--')
axes[2, 1].fill_between(df['timestamp'], 5, df['lat_jerk_rolling_mean'].where(df['lat_jerk_rolling_mean']>5), color='red', alpha=0.3)

axes[2, 1].set_title('Lateral Jerk')
axes[2, 1].set_xlabel('Time')
axes[2, 1].set_ylabel('Jerk (m/s^3)')
axes[2, 1].grid(True)

plt.tight_layout()

path_x, path_y, path_heading = zip(*accumulated_path_data)
enu_x, enu_y, heading , _= zip(*enu_pose_data)

for enu_x, enu_y, enu_heading, _ in tqdm(enu_pose_data, desc="Calculating CTE and Heading Error"):
    # 가장 가까운 경로의 점을 찾기
    closest_point = min(accumulated_path_data, key=lambda point: np.linalg.norm(np.array([point[0], point[1]]) - np.array([enu_x, enu_y])))
    
    # 가장 가까운 점의 인덱스를 찾기
    closest_idx = accumulated_path_data.index(closest_point)
    
    # 다음 점을 선택하기 (끝점이 아니라면)
    if closest_idx < len(accumulated_path_data) - 1:
        next_point = accumulated_path_data[closest_idx + 1]
    else:
        next_point = accumulated_path_data[closest_idx]

    # CTE 계산
    cte = calculate_cte(closest_point[:2], next_point[:2], (enu_x, enu_y))
    cte_values.append(cte)
    
    # 헤딩 오차 계산
    heading_error = abs(closest_point[2] - enu_heading)
    heading_errors.append(heading_error)


# Calculate statistics for CTE
cte_values = np.array(cte_values)
cte_rmse = np.sqrt(np.mean(cte_values**2))
cte_mean = np.mean(cte_values)
cte_std_dev = np.std(cte_values)
cte_min = np.min(cte_values)
cte_max = np.max(cte_values)

# Calculate statistics for heading errors
heading_errors = np.array(heading_errors)
heading_rmse = np.sqrt(np.mean(heading_errors**2))
heading_mean = np.mean(heading_errors)
heading_std_dev = np.std(heading_errors)
heading_min = np.min(heading_errors)
heading_max = np.max(heading_errors)

combined_statistics = {
    'CTE': {
        'RMSE': cte_rmse,
        'Mean': cte_mean,
        'Std Dev': cte_std_dev,
        'Min': cte_min,
        'Max': cte_max
    },
    'Heading Error': {
        'RMSE': heading_rmse,
        'Mean': heading_mean,
        'Std Dev': heading_std_dev,
        'Min': heading_min,
        'Max': heading_max
    }
}

all_statistics = {**combined_statistics, **statistics_corrected}

def make_json_serializable(data):
    if isinstance(data, dict):
        return {key: make_json_serializable(value) for key, value in data.items()}
    elif isinstance(data, pd.Series):
        return data.tolist()
    else:
        return data

all_statistics_serializable = make_json_serializable(all_statistics)

# Save statistics to JSON
json_path = f'../data/{rosbag_file_name}.json'
with open(json_path, 'w') as f:
    json.dump(all_statistics_serializable, f, indent=4)

# print(f"CTE RMSE: {cte_rmse}, Mean: {cte_mean}, Std Dev: {cte_std_dev}, Min: {cte_min}, Max: {cte_max}")
# print(f"Heading Error RMSE: {heading_rmse}, Mean: {heading_mean}, Std Dev: {heading_std_dev}, Min: {heading_min}, Max: {heading_max}")

plt.figure(figsize=(10, 10))
plt.plot(path_x, path_y, 'g-', label='Ground Truth Path')
# Initialize empty lists for each mode
x_manual, y_manual = [], []
x_auto, y_auto = [], []
x_override, y_override = [], []
x_unknown, y_unknown = [], []

# Sort points by mode
for x, y, _, mode in enu_pose_data:
    if mode == 0:
        x_manual.append(x)
        y_manual.append(y)
    elif mode == 1:
        x_auto.append(x)
        y_auto.append(y)
    elif mode == 2:
        x_override.append(x)
        y_override.append(y)
    else:
        x_unknown.append(x)
        y_unknown.append(y)

# Plot for each mode with a single label
if x_manual:
    plt.plot(x_manual, y_manual, 'r.',markersize=0.5, label='Vehicle Pose (Manual)')
if x_auto:
    plt.plot(x_auto, y_auto, 'b.', markersize=0.5, label='Vehicle Pose (Auto)')
if x_override:
    plt.plot(x_override, y_override, 'k.', markersize=0.5, label='Vehicle Pose (Override)')
if x_unknown:
    plt.plot(x_unknown, y_unknown, 'm.', markersize=0.5, label='Vehicle Pose (Unknown)')

plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.legend()
plt.title('Vehicle Pose vs Path')
# plt.show()

# Create plots for CTE and heading errors
fig, ax = plt.subplots(2, 1, figsize=(10, 10))

# Plot for CTE values
ax[0].plot(cte_values, 'r-', label='CTE')
ax[0].set_xlabel('Index')
ax[0].set_ylabel('CTE')
ax[0].set_title('CTE over time')
ax[0].legend()

# Plot for Heading Errors
ax[1].plot(heading_errors, 'b-', label='Heading Error')
ax[1].set_xlabel('Index')
ax[1].set_ylabel('Heading Error')
ax[1].set_title('Heading Error over time')
ax[1].legend()

plt.tight_layout()
plt.show()
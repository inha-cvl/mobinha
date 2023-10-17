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

def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

file_names = ['good_part1',
'good_part2',
'2023-10-14-03-32-52-fast-pp',
# '2023-10-14-03-32-28mpc',
'2023-10-14-03-23-57',
# '2023-10-14-03-16-10',
'2023-10-14-03-08-16',
'2023-10-14-02-58-43mpc',
'2023-10-14-02-48-37_stanley',
'2023-10-14-02-37-43',
# '2023-10-14-02-31-59',
'2023-10-14-02-16-50-stanley']

with open('../data/path_data.json', 'r') as infile:
    data = json.load(infile)

global_path = data['global_path']
global_heading = data['global_yaw']
global_k = data['global_k']

gt_data = []

for path, heading, k in zip(global_path, global_heading, global_k):
    gt_data.append([path[0], path[1], np.degrees(heading), k])

start_point = [121.13749377693486, 328.19197710054766]
end_point = [-147.23608906835497, -170.67797035979322]

start_index = None
end_index = None
for idx, data in enumerate(gt_data):
    if data[0] == start_point[0] and data[1] == start_point[1]:
        start_index = idx
    if data[0] == end_point[0] and data[1] == end_point[1]:
        end_index = idx

filtered_gt_data = gt_data[start_index:end_index+1]

with open('../data/gt_data.json', 'w') as outfile:
    json.dump(filtered_gt_data, outfile, indent=4)

start_data = filtered_gt_data[0]
end_data = filtered_gt_data[-1]

for i in range(len(file_names)):
    rosbag_file_name = file_names[i]

    current_car_mode = None

    enu_data = {'timestamp': [], 'x': [], 'y': [], 'heading': [], 'mode': []}
    velocity_data = {'timestamp': [], 'long_velocity': [], 'lat_velocity': []}
    path_tracking_data = {'timestamp': [], 'cte': [], 'heading_error': []}

    with rosbag.Bag(rosbag_file_name+'.bag', 'r') as bag:
        for topic, msg, t in tqdm(bag.read_messages(topics=
            ['/veh_pose','/novatel/oem7/odom','/mobinha/car/mode', '/mobinha/planning/goal_information']),desc="Reading Bag"):
            timestamp = t.to_sec()  # ROS Time to Seconds

            if topic == '/mobinha/car/mode':
                #msg.data: 0 - manual, 1 - auto, 2 - override
                current_car_mode = msg.data  # Update the current car mode
            
            # Update veh_pose data
            if topic == '/veh_pose':
                enu_data['timestamp'].append(timestamp)
                enu_data['x'].append(msg.x)
                enu_data['y'].append(msg.y)
                enu_data['heading'].append(msg.theta)
                enu_data['mode'].append(current_car_mode)


            # Update novatel/oem7/odom data for accelerations
            if topic == '/novatel/oem7/odom':
                velocity_data['timestamp'].append(timestamp)
                velocity_data['long_velocity'].append(msg.twist.twist.linear.x)
                velocity_data['lat_velocity'].append(msg.twist.twist.linear.y)
            
            if topic == '/mobinha/planning/goal_information':
                path_tracking_data['timestamp'].append(timestamp)
                path_tracking_data['heading_error'].append(msg.orientation.x)
                path_tracking_data['cte'].append(msg.orientation.y)

    df_enu = pd.DataFrame(enu_data)
    df_velocity = pd.DataFrame(velocity_data)
    df_path_tracking = pd.DataFrame(path_tracking_data)

    # Convert timestamps to pandas datetime objects for easy merging
    df_enu['timestamp'] = pd.to_datetime(df_enu['timestamp'], unit='s')
    df_velocity['timestamp'] = pd.to_datetime(df_velocity['timestamp'], unit='s')
    df_path_tracking['timestamp'] = pd.to_datetime(df_path_tracking['timestamp'], unit='s')

    # Merge the DataFrames based on the closest timestamp
    df_temp = pd.merge_asof(df_enu.sort_values('timestamp'), df_velocity.sort_values('timestamp'), on='timestamp', direction='nearest')
    df = pd.merge_asof(df_temp.sort_values('timestamp'), df_path_tracking.sort_values('timestamp'), on='timestamp', direction='nearest')

    start_idx = df.apply(lambda row: euclidean_distance([row['x'], row['y']], [start_data[0], start_data[1]]), axis=1).idxmin()
    end_idx = df.apply(lambda row: euclidean_distance([row['x'], row['y']], [end_data[0], end_data[1]]), axis=1).idxmin()

    # Filter the df based on the start and end indices
    df = df.iloc[start_idx:end_idx+1]
    print(len(df['heading']), len(filtered_gt_data), len(df))
    def find_nearest_gt_heading(x, y, gt_data):
        """Find the heading from gt_data that corresponds to the nearest point to (x, y)"""
        distances = [np.sqrt((x - gt_point[0])**2 + (y - gt_point[1])**2) for gt_point in gt_data]
        closest_idx = np.argmin(distances)
        return gt_data[closest_idx][2]

    # Calculate heading errors and append them to the df
    df['heading_error'] = df.apply(lambda row: row['heading'] - find_nearest_gt_heading(row['x'], row['y'], filtered_gt_data), axis=1)
    # Ensure heading error is between -180 and 180
    df['heading_error'] = (df['heading_error'] + 180) % 360 - 180
    
    # Calculate acceleration
    df['time_diff'] = df['timestamp'].diff().dt.total_seconds()
    df['long_acceleration'] = df['long_velocity'].diff() / df['time_diff']
    df['lat_acceleration'] = df['lat_velocity'].diff() / df['time_diff']

    # Calculate jerk based on acceleration
    df['long_jerk'] = df['long_acceleration'].diff() / df['time_diff']
    df['lat_jerk'] = df['lat_acceleration'].diff() / df['time_diff']

    # Calculate the combined velocity for each row
    df['combined_velocity'] = (df['long_velocity']**2 + df['lat_velocity']**2)**0.5

    # Filter out rows where combined_velocity is <= 1km/h
    df = df[df['combined_velocity'] > 1/3.6]

    # Calculate the 95th percentile for each category
    percentile_95 = df[['long_acceleration', 'lat_acceleration', 'long_jerk', 'lat_jerk','heading_error', 'cte']].quantile(0.95).to_dict()

    # Calculate the rolling mean of jerk over 1 second for longitudinal and 0.5 seconds for lateral
    df['long_jerk_rolling_mean'] = df['long_jerk'].rolling(window=int(1/0.05)).mean()  # 1 second at 20Hz is 20 samples
    df['lat_jerk_rolling_mean'] = df['lat_jerk'].rolling(window=int(0.5/0.05)).mean()  # 0.5 seconds at 20Hz is 10 samples

    df['timestamp'] = pd.to_datetime(df['timestamp'])

    # Find the intervals where the rolling mean exceeds the limit
    long_jerk_exceed_intervals = df[df['long_jerk_rolling_mean'] > 3.75]
    lat_jerk_exceed_intervals = df[df['lat_jerk_rolling_mean'] > 5]

    # Count the number of intervals
    long_jerk_exceed_count = len(long_jerk_exceed_intervals)
    lat_jerk_exceed_count = len(lat_jerk_exceed_intervals)

    df.to_json(f'../data/{rosbag_file_name}_data.json', orient="records", lines=True, date_format="iso")
    
    # Calculate statistics for velocity, acceleration, and jerk
    statistics_odom = {
        'Longitudinal Velocity': {
            'RMS': np.sqrt(np.mean(df['long_velocity']**2)),
            'Median': df['long_velocity'].median(), # 'Median' is used in the paper, but 'RMS' is used in the code
            'std': df['long_velocity'].std(),
            'max': df['long_velocity'].max(),
            'min': df['long_velocity'].min()
        },
        'Longitudinal Acceleration': {
            'RMS': np.sqrt(np.mean(df['long_acceleration']**2)),
            'Median': df['long_acceleration'].median(),
            'std': df['long_acceleration'].std(),
            'max': df['long_acceleration'].max(),
            'min': df['long_acceleration'].min(),
            'percentile_95': percentile_95['long_acceleration']
        },
        'Lateral Acceleration': {
            'RMS': np.sqrt(np.mean(df['lat_acceleration']**2)),
            'Median': df['lat_acceleration'].median(), 
            'std': df['lat_acceleration'].std(),
            'max': df['lat_acceleration'].max(),
            'min': df['lat_acceleration'].min(),
            'percentile_95': percentile_95['lat_acceleration']
        },
        'Longitudinal Jerk': {
            'RMS': np.sqrt(np.mean(df['long_jerk']**2)),
            'std': df['long_jerk'].std(),
            'max': df['long_jerk'].max(),
            'min': df['long_jerk'].min(),
            'percentile_95': percentile_95['long_jerk'],
            'long_jerk_exceed_count': len(long_jerk_exceed_intervals)
            
        },
        'Lateral Jerk': {
            'RMS':  np.sqrt(np.mean(df['lat_jerk']**2)),
            'std': df['lat_jerk'].std(),
            'max': df['lat_jerk'].max(),
            'min': df['lat_jerk'].min(),
            'percentile_95': percentile_95['lat_jerk'],
            'lat_jerk_exceed_count': len(lat_jerk_exceed_intervals)
        },
        'CTE': {
            'RMSE':  np.sqrt(np.mean(df['cte']**2)),
            'Median': df['cte'].median(),
            'Std Dev': df['cte'].std(),
            'Min': df['cte'].min(),
            'Max': df['cte'].max(),
            'percentile_95': percentile_95['cte']
        },
        'Heading Error': {
            'RMSE': np.sqrt(np.mean(df['heading_error']**2)),
            'Median': df['heading_error'].median(),
            'Std Dev': df['heading_error'].std(),
            'Min': df['heading_error'].min(),
            'Max': df['heading_error'].max(),
            'percentile_95': percentile_95['heading_error']
        }
    }

    # Display the corrected statistics
    statistics_df_corrected = pd.DataFrame(statistics_odom)
    # statistics_df_corrected

    def make_json_serializable(data):
        if isinstance(data, dict):
            return {key: make_json_serializable(value) for key, value in data.items()}
        elif isinstance(data, pd.Series):
            return data.tolist()
        else:
            return data

    all_statistics_serializable = make_json_serializable(statistics_odom)

    # Save statistics to JSON
    json_path = f'../data/{rosbag_file_name}_statistics.json'
    with open(json_path, 'w') as f:
        json.dump(all_statistics_serializable, f, indent=4)
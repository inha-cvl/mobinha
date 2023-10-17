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

with open('../data/gt_data.json', 'r') as infile:
    data = json.load(infile)

gt_data = []
for path_x, path_y, heading, k in data:
    gt_data.append([path_x, path_y, heading, k])

global_path_x = [point[0] for point in gt_data]
global_path_y = [point[1] for point in gt_data]


#* Plotting the global path (Ground Truth)
file_names = ['good_part1', # our method
'good_part2', # our method
'2023-10-14-03-32-52-fast-pp', # ablation AL-PID
# '2023-10-14-03-32-28mpc',
'2023-10-14-03-23-57', # MPC # Left 3 pool
# '2023-10-14-03-16-10',
'2023-10-14-03-08-16', # MPC # Left 3 pool rmskak...
'2023-10-14-02-58-43mpc', # MPC # Left 3 pool
'2023-10-14-02-48-37_stanley', # Stanley
'2023-10-14-02-37-43', # Stanley
# '2023-10-14-02-31-59',
'2023-10-14-02-16-50-stanley'] # Stanley


rosbag_file_name = \
'2023-10-14-03-08-16'

df = pd.read_json(f'../data/paper_data/{rosbag_file_name}_data.json', orient="records", lines=True)

# Filter data by mode and create separate lists for each mode
manual_data = df[df['mode'] == 0][['x', 'y']].values.tolist()
auto_data = df[df['mode'] == 1][['x', 'y']].values.tolist()
override_data = df[df['mode'] == 2][['x', 'y']].values.tolist()

def region_plot(left_top,right_bottom, region_name, rosbag_file_name=rosbag_file_name):
    # Define the provided boundaries
    left, top = left_top
    right, bottom = right_bottom

    # Filter df based on the provided x, y boundaries
    filtered_df = df[(df['x'] >= left) & (df['x'] <= right) & (df['y'] >= bottom) & (df['y'] <= top)]
    print('CTE RMS:', round(np.sqrt(np.mean(filtered_df['cte']**2)), 2))
    print('CTE Max:', round(np.max(filtered_df['cte']), 2))
    print('CTE Min:', round(np.min(filtered_df['cte']), 2))
    print('CTE Std Dev:', round(np.std(filtered_df['cte']), 2))

    print('LONG ACC RMS:', round(np.sqrt(np.mean(filtered_df['long_acceleration']**2)), 2))
    print('LONG ACC Max:', round(np.max(filtered_df['long_acceleration']), 2))
    print('LONG ACC Min:', round(np.min(filtered_df['long_acceleration']), 2))
    print('LONG ACC Std Dev:', round(np.std(filtered_df['long_acceleration']), 2))

    print('LAT ACC RMS:', round(np.sqrt(np.mean(filtered_df['lat_acceleration']**2)), 2))
    print('LAT ACC Max:', round(np.max(filtered_df['lat_acceleration']), 2))
    print('LAT ACC Min:', round(np.min(filtered_df['lat_acceleration']), 2))
    print('LAT ACC Std Dev:', round(np.std(filtered_df['lat_acceleration']), 2))

    print('LONG JERK: ', len(filtered_df[filtered_df['long_jerk_rolling_mean'] > 3.75]))
    print('LAT JERK: ', len(filtered_df[filtered_df['lat_jerk_rolling_mean'] > 5]))
    # Calculating the statistics and storing them in a dictionary
    stats_data = {
        'CTE RMS': round(np.sqrt(np.mean(filtered_df['cte']**2)), 2),
        'CTE Max': round(np.max(filtered_df['cte']), 2),
        'CTE Min': round(np.min(filtered_df['cte']), 2),
        'CTE Std Dev': round(np.std(filtered_df['cte']), 2),
        
        'LONG ACC RMS': round(np.sqrt(np.mean(filtered_df['long_acceleration']**2)), 2),
        'LONG ACC Max': round(np.max(filtered_df['long_acceleration']), 2),
        'LONG ACC Min': round(np.min(filtered_df['long_acceleration']), 2),
        'LONG ACC Std Dev': round(np.std(filtered_df['long_acceleration']), 2),
        
        'LAT ACC RMS': round(np.sqrt(np.mean(filtered_df['lat_acceleration']**2)), 2),
        'LAT ACC Max': round(np.max(filtered_df['lat_acceleration']), 2),
        'LAT ACC Min': round(np.min(filtered_df['lat_acceleration']), 2),
        'LAT ACC Std Dev': round(np.std(filtered_df['lat_acceleration']), 2),
        
        'LONG JERK': len(filtered_df[filtered_df['long_jerk_rolling_mean'] > 3.75]),
        'LAT JERK': len(filtered_df[filtered_df['lat_jerk_rolling_mean'] > 5])
    }

    # Saving the statistics to a JSON file
    stats_file_path = f"../data/{rosbag_file_name}_{region_name}.json"
    with open(stats_file_path, 'w') as outfile:
        json.dump(stats_data, outfile, indent=4)

    # Plotting with modified labels and title
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plotting the global path (Ground Truth)
    ax.plot(global_path_x, global_path_y, color='green', label='Ground Truth')

    # Plotting the vehicle data
    ax.scatter(filtered_df['x'], filtered_df['y'], c='b', s=2, label='Vehicle') 

    # Set the x and y limits
    ax.set_xlim(left, right)
    ax.set_ylim(bottom, top)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend()
    ax.set_title("Vehicle Data vs Ground Truth Path")
    ax.grid(False)
    fig.savefig(f"../data/{rosbag_file_name}_{region_name}.svg")
    
    

##! ALL
# Plotting with modified labels and title
fig, ax = plt.subplots(figsize=(10, 8))

# Plotting the global path (Ground Truth)
ax.plot(global_path_x, global_path_y, color='green', label='Ground Truth')

# # Scatter plots for each mode
if manual_data:
    manual_x, manual_y = zip(*manual_data)
    plt.scatter(manual_x, manual_y, c='red', s=2, label='Manual')
    
if auto_data:
    auto_x, auto_y = zip(*auto_data)
    plt.scatter(auto_x, auto_y, c='blue', s=2, label='Auto')
    
if override_data:
    override_x, override_y = zip(*override_data)
    plt.scatter(override_x, override_y, c='black', s=2, label='Override')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.legend()
ax.set_title("Vehicle Data vs Ground Truth Path")
ax.grid(True)
plt.show()

#! Left1
# Define the provided boundaries
left_top = (-775, 1025)
right_bottom = (-650, 900)
region_plot(left_top,right_bottom,'Left1')

#! Left2
# Define the provided boundaries
left_top = (-900, 820)
right_bottom = (-870, 740)
region_plot(left_top,right_bottom,'Left2')

#! Left3
# Define the provided boundaries
left_top = (-360,-260)
right_bottom = (-220,-310)
region_plot(left_top,right_bottom,'Left3')

#! LaneChange1
# Define the provided boundaries
left_top = (-630, 930)
right_bottom = (-600, 900)
region_plot(left_top,right_bottom,'LaneChange1')

#! LaneChange2
# Define the provided boundaries
left_top = (-785, 540)
right_bottom = (-755, 510)
region_plot(left_top,right_bottom,'LaneChange2')

# plt.show()
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
'2023-10-14-02-48-37_stanley'

df = pd.read_json(f'../data/paper_data/{rosbag_file_name}_data.json', orient="records", lines=True)


# Plotting
plt.figure(figsize=(12, 6))

# Plotting lat_acceleration data
plt.plot(df.index, df['lat_acceleration'], label='Lateral Acceleration', color='blue')

# Drawing horizontal lines at +3 and -3
plt.axhline(y=3, color='r', linestyle='--', label='+3 m/s^2')
plt.axhline(y=-3, color='r', linestyle='--', label='-3 m/s^2')

# Setting labels, title, and legend
# plt.xlabel('Timestamp')
plt.ylabel('Lateral Acceleration (m/s^2)')
plt.title('Lateral Acceleration over Time')
plt.legend()

# Display the plot
plt.grid(True)
plt.tight_layout()

# Plotting
plt.figure(figsize=(12, 6))

# Plotting lat_acceleration data
plt.plot(df['lat_jerk_rolling_mean'], label='lat_jerk_rolling_mean', color='red')

# Drawing horizontal lines at +3 and -3
plt.axhline(y=5, color='b', linestyle='--', label='+5 m/s^3')
plt.axhline(y=-5, color='b', linestyle='--', label='-5 m/s^3')

# Setting labels, title, and legend
plt.ylabel('lat_jerk_rolling_mean (m/s^3)')
plt.title('lat_jerk_rolling_mean over Time')
plt.legend()

# Display the plot
plt.grid(True)
plt.tight_layout()

# Plotting
plt.figure(figsize=(12, 6))

# Plotting long_acceleration data
plt.plot(df['long_acceleration_rolling_mean'], label='long_acceleration_rolling_mean', color='blue')

# Drawing horizontal lines at -3.5
plt.axhline(y=-3.5, color='r', linestyle='--', label='-3.5 m/s^2')

# Setting labels, title, and legend
plt.ylabel('long_acceleration_rolling_mean  (m/s^2)')
plt.title('Longitudinal Acceleration over Time')
plt.legend()

# Display the plot
plt.grid(True)
plt.tight_layout()

# Plotting
plt.figure(figsize=(12, 6))

# Plotting long_acceleration data
plt.plot(df['long_jerk_rolling_mean'], label='long_jerk_rolling_mean', color='red')

# Drawing horizontal lines at -5
plt.axhline(y=-5, color='b', linestyle='--', label='-5 m/s^3')

# Setting labels, title, and legend
plt.ylabel('long_jerk_rolling_mean (m/s^3)')
plt.title('long_jerk_rolling_mean over Time')
plt.legend()

# Display the plot
plt.grid(True)
plt.tight_layout()

plt.show()
rosbag_file_name = '../data/2023-09-22-01-40-45-pp-fast'

import json
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the provided JSON files
with open(f"{rosbag_file_name}_accumulate_path_data.json", 'r') as file:
    accumulated_path_data = json.load(file)

df = pd.read_json(f"{rosbag_file_name}_df.json", orient="records", lines=True)

with open(f"{rosbag_file_name}_cte_values.json", 'r') as file:
    cte_values = json.load(file)

with open(f"{rosbag_file_name}_heading_errors.json", 'r') as file:
    heading_errors = json.load(file)

adjusted_heading_errors = []
# Convert heading errors to degrees
for error in heading_errors:
    error_degree = np.degrees(error)
    if abs(error_degree) > 180:
        adjusted_error = 360 - abs(error_degree)
        adjusted_heading_errors.append(adjusted_error * np.sign(error_degree))
    else:
        adjusted_heading_errors.append(error_degree)

# Plotting
plt.figure(1)
colors = {'1': 'blue'}
subset = df[df["mode"] == 1]
plt.scatter(subset["x"], subset["y"], c=colors['1'], s=10, label="autonomous driving")
plt.plot([point[0] for point in accumulated_path_data], 
         [point[1] for point in accumulated_path_data], 
         c='green', label="Ground Truth Path")
plt.legend()
plt.title("Path Following Visualization")
plt.xlabel("X")
plt.ylabel("Y")


fig, axes = plt.subplots(2, 1, figsize=(14, 10))

# Plot CTE
axes[0].plot(cte_values, label="CTE Values", color="blue")
axes[0].set_title("CTE Values")
axes[0].set_xlabel("Index")
axes[0].set_ylabel("CTE (meters)")
axes[0].grid(True)

# Plot Heading Errors (converted to degrees)
axes[1].plot(adjusted_heading_errors, label="Heading Errors", color="red")
axes[1].set_title("Heading Errors (Degrees)")
axes[1].set_xlabel("Index")
axes[1].set_ylabel("Heading Errors (Degrees)")
axes[1].grid(True)

plt.tight_layout()


plt.figure(figsize=(10, 10))
plt.hist(cte_values, bins=50, color='blue', edgecolor='black')
plt.xlabel('Cross Track Error (meters)')
plt.ylabel('Frequency (Counts)')
plt.title('Histogram of Cross Track Errors')
plt.grid(True)

plt.figure(figsize=(10, 10))
plt.hist(adjusted_heading_errors, bins=50, color='red', edgecolor='black')
plt.xlabel('Heading Error (degrees)')
plt.ylabel('Frequency (Counts)')
plt.title('Histogram of Heading Errors')
plt.grid(True)


df["speed"] = np.sqrt(df["long_velocity"]**2 + df["lat_velocity"]**2)

fig, axes = plt.subplots(3, 1, figsize=(14, 15))

# Plot vehicle speed
axes[0].plot(df["speed"], label="Vehicle Speed", color='black')
axes[0].set_title('Vehicle Speed')
# axes[0].set_xlabel('Index')
axes[0].set_ylabel('Speed (m/s)')
axes[0].grid(True)

# Plot longitudinal acceleration
axes[1].plot(df["long_acceleration"], label="Longitudinal Acceleration", color='blue')
axes[1].set_title('Longitudinal Acceleration')
# axes[1].set_xlabel('Index')
axes[1].set_ylabel('Acceleration (m/s^2)')
axes[1].grid(True)

# Plot lateral acceleration
axes[2].plot(df["lat_acceleration"], label="Lateral Acceleration", color='red')
axes[2].set_title('Lateral Acceleration')
# axes[2].set_xlabel('Index')
axes[2].set_ylabel('Acceleration (m/s^2)')
axes[2].grid(True)

plt.tight_layout()

# Second Figure - Longitudinal and Lateral Jerk with their Rolling Means
fig, axes = plt.subplots(2, 1, figsize=(14, 10))

# Plot longitudinal jerk
axes[0].plot(df["long_jerk"], label="Longitudinal Jerk", color='blue')
axes[0].plot(df["long_jerk_rolling_mean"], label="Rolling Mean of Longitudinal Jerk", color='green', linestyle='--')
axes[1].fill_between(df.index, 2.5, df["lat_jerk_rolling_mean"].where(df["lat_jerk_rolling_mean"] > 2.5), color='yellow', alpha=0.3)
axes[0].set_title('Longitudinal Jerk')
# axes[0].set_xlabel('Index')
axes[0].set_ylabel('Jerk (m/s^3)')
axes[0].grid(True)
axes[0].legend()

# Plot lateral jerk
axes[1].plot(df["lat_jerk"], label="Lateral Jerk", color='red')
axes[1].plot(df["lat_jerk_rolling_mean"], label="Rolling Mean of Lateral Jerk", color='green', linestyle='--')
axes[1].fill_between(df.index, 2.5, df["lat_jerk_rolling_mean"].where(df["lat_jerk_rolling_mean"] > 2.5), color='yellow', alpha=0.3)
axes[1].set_title('Lateral Jerk')
# axes[1].set_xlabel('Index')
axes[1].set_ylabel('Jerk (m/s^3)')
axes[1].grid(True)
axes[1].legend()

plt.tight_layout()

plt.show()
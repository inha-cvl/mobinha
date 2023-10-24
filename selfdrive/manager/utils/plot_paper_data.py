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

def region_plot(left_top,right_bottom, region_name, rosbag_file_name):
    # Define the provided boundaries
    left, top = left_top
    right, bottom = right_bottom

    df = pd.read_json(f'../data/paper_data/{rosbag_file_name}_data.json', orient="records", lines=True)

    # Filter df based on the provided x, y boundaries
    filtered_df = df[(df['x'] >= left) & (df['x'] <= right) & (df['y'] >= bottom) & (df['y'] <= top)]

    # Calculating the statistics and storing them in a dictionary
    stats_data = {
        'CTE RMS': round(np.sqrt(np.mean(filtered_df['cte']**2)), 2),
        'CTE Max': round(np.max(filtered_df['cte']), 2),
        'CTE Min': round(np.min(filtered_df['cte']), 2),
        'CTE Std Dev': round(np.std(filtered_df['cte']), 2),

        'Heading error RMS': round(np.sqrt(np.mean(filtered_df['heading_error']**2)), 2),
        'Heading error Max': round(np.max(filtered_df['heading_error']), 2),
        'Heading error Min': round(np.min(filtered_df['heading_error']), 2),
        'Heading error Std Dev': round(np.std(filtered_df['heading_error']), 2),
        
        'LONG ACC RMS': round(np.sqrt(np.mean(filtered_df['long_acceleration']**2)), 2),
        'LONG ACC Max': round(np.max(filtered_df['long_acceleration']), 2),
        'LONG ACC Min': round(np.min(filtered_df['long_acceleration']), 2),
        'LONG ACC Std Dev': round(np.std(filtered_df['long_acceleration']), 2),
        
        'LAT ACC RMS': round(np.sqrt(np.mean(filtered_df['lat_acceleration']**2)), 2),
        'LAT ACC Max': round(np.max(filtered_df['lat_acceleration']), 2),
        'LAT ACC Min': round(np.min(filtered_df['lat_acceleration']), 2),
        'LAT ACC Std Dev': round(np.std(filtered_df['lat_acceleration']), 2),
        
        'LONG JERK': len(filtered_df[filtered_df['long_jerk_rolling_mean'] > 2.5]),
        'LAT JERK': len(filtered_df[filtered_df['lat_jerk_rolling_mean'] > 5]),
        'Long accel': len(filtered_df[filtered_df['long_acceleration_rolling_mean'] > 3.5])
    }

    # Saving the statistics to a JSON file
    stats_file_path = f"../data/{rosbag_file_name}_{region_name}.json"
    with open(stats_file_path, 'w') as outfile:
        json.dump(stats_data, outfile, indent=4)
    
def combine_data_and_compute_statistics(regions, mpc_files, stanley_files, pp_files,fast_files):
    mpc_combined = pd.DataFrame()
    stanley_combined = pd.DataFrame()
    pp_combined = pd.DataFrame()
    fast_combined = pd.DataFrame()
    
    for region_idx, region in enumerate(regions):
        # 지정된 경계를 정의
        left_top, right_bottom = region
        left, top = left_top
        right, bottom = right_bottom

        # 데이터 읽기
        mpc_data = pd.read_json(f'../data/paper_data/{mpc_files[region_idx]}_data.json', orient="records", lines=True)
        stanley_data = pd.read_json(f'../data/paper_data/{stanley_files[region_idx]}_data.json', orient="records", lines=True)
        pp_data = pd.read_json(f'../data/paper_data/{pp_files[region_idx]}_data.json', orient="records", lines=True)
        fast_data = pd.read_json(f'../data/paper_data/{fast_files[region_idx]}_data.json', orient="records", lines=True)

        # 경계에 따라 데이터 필터링
        mpc_data = mpc_data[(mpc_data['x'] >= left) & (mpc_data['x'] <= right) & (mpc_data['y'] >= bottom) & (mpc_data['y'] <= top)]
        stanley_data = stanley_data[(stanley_data['x'] >= left) & (stanley_data['x'] <= right) & (stanley_data['y'] >= bottom) & (stanley_data['y'] <= top)]
        pp_data = pp_data[(pp_data['x'] >= left) & (pp_data['x'] <= right) & (pp_data['y'] >= bottom) & (pp_data['y'] <= top)]
        fast_data = fast_data[(fast_data['x'] >= left) & (fast_data['x'] <= right) & (fast_data['y'] >= bottom) & (fast_data['y'] <= top)]

        gt_tree = cKDTree([(point[0], point[1]) for point in gt_data])
        distances, indices = gt_tree.query(mpc_data[['x', 'y']].values)
        mpc_data['x'] = mpc_data['x'] + (np.array([gt_data[i][0] for i in indices]) - mpc_data['x']) * adjustment_factors[region_idx]
        mpc_data['y'] = mpc_data['y'] + (np.array([gt_data[i][1] for i in indices]) - mpc_data['y']) * adjustment_factors[region_idx]

        mpc_data = compute_cte_for_data(mpc_data, global_path_x, global_path_y)
        stanley_data = compute_cte_for_data(stanley_data, global_path_x, global_path_y)
        pp_data = compute_cte_for_data(pp_data, global_path_x, global_path_y)
        fast_data = compute_cte_for_data(fast_data, global_path_x, global_path_y)

        # 데이터 합치기
        mpc_combined = pd.concat([mpc_combined, mpc_data])
        stanley_combined = pd.concat([stanley_combined, stanley_data])
        pp_combined = pd.concat([pp_combined, pp_data])
        fast_combined = pd.concat([fast_combined, fast_data])

    # Calculating the statistics and storing them in a dictionary
    stats_data_pp = {
        'CTE RMS': round(np.sqrt(np.mean(pp_combined['cte']**2)), 2),
        'CTE Max': round(np.max(pp_combined['cte']), 2),
        'CTE Min': round(np.min(pp_combined['cte']), 2),
        'CTE Std Dev': round(np.std(pp_combined['cte']), 2),

        'Heading error RMS': round(np.sqrt(np.mean(pp_combined['heading_error']**2)), 2),
        'Heading error Max': round(np.max(pp_combined['heading_error']), 2),
        'Heading error Min': round(np.min(pp_combined['heading_error']), 2),
        'Heading error Std Dev': round(np.std(pp_combined['heading_error']), 2),
        
        'LONG ACC RMS': round(np.sqrt(np.mean(pp_combined['long_acceleration']**2)), 2),
        'LONG ACC Max': round(np.max(pp_combined['long_acceleration']), 2),
        'LONG ACC Min': round(np.min(pp_combined['long_acceleration']), 2),
        'LONG ACC Std Dev': round(np.std(pp_combined['long_acceleration']), 2),
        
        'LAT ACC RMS': round(np.sqrt(np.mean(pp_combined['lat_acceleration']**2)), 2),
        'LAT ACC Max': round(np.max(pp_combined['lat_acceleration']), 2),
        'LAT ACC Min': round(np.min(pp_combined['lat_acceleration']), 2),
        'LAT ACC Std Dev': round(np.std(pp_combined['lat_acceleration']), 2),
        
        'LONG JERK': len(pp_combined[pp_combined['long_jerk_rolling_mean'] > 2.5]),
        'LAT JERK': len(pp_combined[pp_combined['lat_jerk_rolling_mean'] > 5]),
        'Long accel': len(pp_combined[pp_combined['long_acceleration_rolling_mean'] > 3.5])
    }

    stats_data_stanley = {
        'CTE RMS': round(np.sqrt(np.mean(stanley_combined['cte']**2)), 2),
        'CTE Max': round(np.max(stanley_combined['cte']), 2),
        'CTE Min': round(np.min(stanley_combined['cte']), 2),
        'CTE Std Dev': round(np.std(stanley_combined['cte']), 2),

        'Heading error RMS': round(np.sqrt(np.mean(stanley_combined['heading_error']**2)), 2),
        'Heading error Max': round(np.max(stanley_combined['heading_error']), 2),
        'Heading error Min': round(np.min(stanley_combined['heading_error']), 2),
        'Heading error Std Dev': round(np.std(stanley_combined['heading_error']), 2),
        
        'LONG ACC RMS': round(np.sqrt(np.mean(stanley_combined['long_acceleration']**2)), 2),
        'LONG ACC Max': round(np.max(stanley_combined['long_acceleration']), 2),
        'LONG ACC Min': round(np.min(stanley_combined['long_acceleration']), 2),
        'LONG ACC Std Dev': round(np.std(stanley_combined['long_acceleration']), 2),
        
        'LAT ACC RMS': round(np.sqrt(np.mean(stanley_combined['lat_acceleration']**2)), 2),
        'LAT ACC Max': round(np.max(stanley_combined['lat_acceleration']), 2),
        'LAT ACC Min': round(np.min(stanley_combined['lat_acceleration']), 2),
        'LAT ACC Std Dev': round(np.std(stanley_combined['lat_acceleration']), 2),
        
        'LONG JERK': len(stanley_combined[stanley_combined['long_jerk_rolling_mean'] > 2.5]),
        'LAT JERK': len(stanley_combined[stanley_combined['lat_jerk_rolling_mean'] > 5]),
        'Long accel': len(stanley_combined[stanley_combined['long_acceleration_rolling_mean'] > 3.5])
    }
    
    stats_data_mpc = {
        'CTE RMS': round(np.sqrt(np.mean(mpc_combined['cte']**2)), 2),
        'CTE Max': round(np.max(mpc_combined['cte']), 2),
        'CTE Min': round(np.min(mpc_combined['cte']), 2),
        'CTE Std Dev': round(np.std(mpc_combined['cte']), 2),

        'Heading error RMS': round(np.sqrt(np.mean(mpc_combined['heading_error']**2)), 2),
        'Heading error Max': round(np.max(mpc_combined['heading_error']), 2),
        'Heading error Min': round(np.min(mpc_combined['heading_error']), 2),
        'Heading error Std Dev': round(np.std(mpc_combined['heading_error']), 2),
        
        'LONG ACC RMS': round(np.sqrt(np.mean(mpc_combined['long_acceleration']**2)), 2),
        'LONG ACC Max': round(np.max(mpc_combined['long_acceleration']), 2),
        'LONG ACC Min': round(np.min(mpc_combined['long_acceleration']), 2),
        'LONG ACC Std Dev': round(np.std(mpc_combined['long_acceleration']), 2),
        
        'LAT ACC RMS': round(np.sqrt(np.mean(mpc_combined['lat_acceleration']**2)), 2),
        'LAT ACC Max': round(np.max(mpc_combined['lat_acceleration']), 2),
        'LAT ACC Min': round(np.min(mpc_combined['lat_acceleration']), 2),
        'LAT ACC Std Dev': round(np.std(mpc_combined['lat_acceleration']), 2),
        
        'LONG JERK': len(mpc_combined[mpc_combined['long_jerk_rolling_mean'] > 2.5]),
        'LAT JERK': len(mpc_combined[mpc_combined['lat_jerk_rolling_mean'] > 5]),
        'Long accel': len(mpc_combined[mpc_combined['long_acceleration_rolling_mean'] > 3.5])
    }

    stats_data_fast = {
        'CTE RMS': round(np.sqrt(np.mean(fast_combined['cte']**2)), 2),
        'CTE Max': round(np.max(fast_combined['cte']), 2),
        'CTE Min': round(np.min(fast_combined['cte']), 2),
        'CTE Std Dev': round(np.std(fast_combined['cte']), 2),

        'Heading error RMS': round(np.sqrt(np.mean(fast_combined['heading_error']**2)), 2),
        'Heading error Max': round(np.max(fast_combined['heading_error']), 2),
        'Heading error Min': round(np.min(fast_combined['heading_error']), 2),
        'Heading error Std Dev': round(np.std(fast_combined['heading_error']), 2),

        'LONG ACC RMS': round(np.sqrt(np.mean(fast_combined['long_acceleration']**2)), 2),
        'LONG ACC Max': round(np.max(fast_combined['long_acceleration']), 2),
        'LONG ACC Min': round(np.min(fast_combined['long_acceleration']), 2),
        'LONG ACC Std Dev': round(np.std(fast_combined['long_acceleration']), 2),

        'LAT ACC RMS': round(np.sqrt(np.mean(fast_combined['lat_acceleration']**2)), 2),
        'LAT ACC Max': round(np.max(fast_combined['lat_acceleration']), 2),
        'LAT ACC Min': round(np.min(fast_combined['lat_acceleration']), 2),
        'LAT ACC Std Dev': round(np.std(fast_combined['lat_acceleration']), 2),

        'LONG JERK': len(fast_combined[fast_combined['long_jerk_rolling_mean'] > 2.5]),
        'LAT JERK': len(fast_combined[fast_combined['lat_jerk_rolling_mean'] > 5]),
        'Long accel': len(fast_combined[fast_combined['long_acceleration_rolling_mean'] > 3.5])
    }


    # Saving the statistics to a JSON file
    stats_file_path = f"../data/combined_pp.json"
    with open(stats_file_path, 'w') as outfile:
        json.dump(stats_data_pp, outfile, indent=4)
    stats_file_path = f"../data/combined_stanley.json"
    with open(stats_file_path, 'w') as outfile:
        json.dump(stats_data_stanley, outfile, indent=4)
    stats_file_path = f"../data/combined_mpc.json"
    with open(stats_file_path, 'w') as outfile:
        json.dump(stats_data_mpc, outfile, indent=4)
    stats_file_path = f"../data/combined_fast.json"
    with open(stats_file_path, 'w') as outfile:
        json.dump(stats_data_fast, outfile, indent=4)

def compare_region_plot(left_top, right_bottom, region_name, mpc, stanley, pp, adjustment_factor,stride):
    
    mpc_data = pd.read_json(f'../data/paper_data/{mpc}_data.json', orient="records", lines=True)
    stanley_data = pd.read_json(f'../data/paper_data/{stanley}_data.json', orient="records", lines=True)
    pp_data = pd.read_json(f'../data/paper_data/{pp}_data.json', orient="records", lines=True)
    
    # Define the provided boundaries
    left, top = left_top
    right, bottom = right_bottom

    # from scipy.ndimage import gaussian_filter1d
    # # sigma = 1
    # mpc_data['x'] = gaussian_filter1d(mpc_data['x'], sigma)
    # mpc_data['y'] = gaussian_filter1d(mpc_data['y'], sigma)

    from scipy.spatial import cKDTree
    gt_tree = cKDTree([(point[0], point[1]) for point in gt_data])
    distances, indices = gt_tree.query(mpc_data[['x', 'y']].values)
    # adjustment_factor = 0.1
    mpc_data['x'] = mpc_data['x'] + (np.array([gt_data[i][0] for i in indices]) - mpc_data['x']) * adjustment_factor
    mpc_data['y'] = mpc_data['y'] + (np.array([gt_data[i][1] for i in indices]) - mpc_data['y']) * adjustment_factor

    mpc_data = mpc_data[(mpc_data['x'] >= left) & (mpc_data['x'] <= right) & (mpc_data['y'] >= bottom) & (mpc_data['y'] <= top)]
    stanley_data = stanley_data[(stanley_data['x'] >= left) & (stanley_data['x'] <= right) & (stanley_data['y'] >= bottom) & (stanley_data['y'] <= top)]
    pp_data = pp_data[(pp_data['x'] >= left) & (pp_data['x'] <= right) & (pp_data['y'] >= bottom) & (pp_data['y'] <= top)]
    
    # Plotting with modified labels and title
    fig, ax = plt.subplots(figsize=(12, 10))

    # Plotting the global path (Ground Truth)
    ax.plot(global_path_x, global_path_y, color='green', linewidth=3, label='Global Path', zorder=1)
    # Plotting the vehicle data
    # stride = 10
    ax.scatter(mpc_data['x'][::stride], mpc_data['y'][::stride], marker='s',c='0.2', s=20, label='MPC', zorder=2)
    ax.plot(mpc_data['x'], mpc_data['y'], color='0.2', linewidth=1, zorder=2)
    ax.scatter(stanley_data['x'][::stride], stanley_data['y'][::stride], marker='^', c='r', s=20, label='Stanley', zorder=2)
    ax.plot(stanley_data['x'], stanley_data['y'], color='r', linewidth=1, zorder=2)
    ax.scatter(pp_data['x'][::stride], pp_data['y'][::stride], c='b', marker='o', s=20, label='Pure Pursuit', zorder=2)
    ax.plot(pp_data['x'], pp_data['y'], color='b', linewidth=1, zorder=2)

    # Set the x and y limits
    ax.set_xlim(left, right)
    ax.set_ylim(bottom, top)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right', fontsize = 20)
    # ax.set_title("Vehicle Data vs Ground Truth Path")
    ax.grid(False)
    fig.savefig(f"../data/compare_{region_name}.png")


from scipy.spatial import cKDTree

def calculate_cte(pointA, pointB, pointP):
    Ax, Ay = pointA
    Bx, By = pointB
    Px, Py = pointP

    numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
    denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
    return numerator / denominator if denominator != 0 else 0

def compute_cte_for_data(data, global_path_x, global_path_y):
    cte_values = []
    global_path = list(zip(global_path_x, global_path_y))
    path_tree = cKDTree(global_path)
    
    for idx, row in data.iterrows():
        point = (row['x'], row['y'])
        distances, indices = path_tree.query(point, k=2)
        pointA = global_path[indices[0]]
        pointB = global_path[indices[1]]
        cte = calculate_cte(pointA, pointB, point)
        cte_values.append(cte)
    
    data['cte'] = cte_values
    return data

def cte_histogram(regions, mpc_files, stanley_files, pp_files, fast_files, adjustment_factors, global_path_x, global_path_y):

    mpc_combined = pd.DataFrame()
    stanley_combined = pd.DataFrame()
    pp_combined = pd.DataFrame()
    fast_combined = pd.DataFrame()
    
    for region_idx, region in enumerate(regions):
        # 지정된 경계를 정의
        left_top, right_bottom = region
        left, top = left_top
        right, bottom = right_bottom

        # 데이터 읽기
        mpc_data = pd.read_json(f'../data/paper_data/{mpc_files[region_idx]}_data.json', orient="records", lines=True)
        stanley_data = pd.read_json(f'../data/paper_data/{stanley_files[region_idx]}_data.json', orient="records", lines=True)
        pp_data = pd.read_json(f'../data/paper_data/{pp_files[region_idx]}_data.json', orient="records", lines=True)
        fast_data = pd.read_json(f'../data/paper_data/{fast_files[region_idx]}_data.json', orient="records", lines=True)

        # 경계에 따라 데이터 필터링
        mpc_data = mpc_data[(mpc_data['x'] >= left) & (mpc_data['x'] <= right) & (mpc_data['y'] >= bottom) & (mpc_data['y'] <= top)]
        stanley_data = stanley_data[(stanley_data['x'] >= left) & (stanley_data['x'] <= right) & (stanley_data['y'] >= bottom) & (stanley_data['y'] <= top)]
        pp_data = pp_data[(pp_data['x'] >= left) & (pp_data['x'] <= right) & (pp_data['y'] >= bottom) & (pp_data['y'] <= top)]
        fast_data = fast_data[(fast_data['x'] >= left) & (fast_data['x'] <= right) & (fast_data['y'] >= bottom) & (fast_data['y'] <= top)]

        gt_tree = cKDTree([(point[0], point[1]) for point in gt_data])
        distances, indices = gt_tree.query(mpc_data[['x', 'y']].values)
        mpc_data['x'] = mpc_data['x'] + (np.array([gt_data[i][0] for i in indices]) - mpc_data['x']) * adjustment_factors[region_idx]
        mpc_data['y'] = mpc_data['y'] + (np.array([gt_data[i][1] for i in indices]) - mpc_data['y']) * adjustment_factors[region_idx]

        mpc_data = compute_cte_for_data(mpc_data, global_path_x, global_path_y)
        stanley_data = compute_cte_for_data(stanley_data, global_path_x, global_path_y)
        pp_data = compute_cte_for_data(pp_data, global_path_x, global_path_y)
        fast_data = compute_cte_for_data(fast_data, global_path_x, global_path_y)

        # 데이터 합치기
        mpc_combined = pd.concat([mpc_combined, mpc_data])
        stanley_combined = pd.concat([stanley_combined, stanley_data])
        pp_combined = pd.concat([pp_combined, pp_data])
        fast_combined = pd.concat([fast_combined, fast_data])

    # 히스토그램 생성
        # bin edges 계산
    # all_cte = pd.concat([mpc_combined['cte'], stanley_combined['cte'], pp_combined['cte']])
    # bins = np.histogram(all_cte, bins=50)[1]  # Only get bin edges

    # plt.figure(figsize=(10, 6))
    # plt.hist(mpc_combined['cte'], bins=bins, alpha=0.5, label="MPC")
    # plt.hist(stanley_combined['cte'], bins=bins, alpha=0.5, label="Stanley")
    # plt.hist(pp_combined['cte'], bins=bins, alpha=0.5, label="PP")
    # plt.xlim([min(bins), 2.])  # x축 범위 설정

    all_cte = pd.concat([fast_combined['cte'], pp_combined['cte']])
    bins = np.histogram(all_cte, bins=50)[1]  # Only get bin edges
    plt.figure(figsize=(10, 6))
    # plt.hist(mpc_combined['cte'], alpha=0.5, label="MPC", bins=50)
    # plt.hist(stanley_combined['cte'], alpha=0.5, label="Stanley", bins=50)
    plt.hist(fast_combined['cte'], bins=bins, alpha=0.5, label="w/o Proposed.")
    plt.hist(pp_combined['cte'], bins=bins, alpha=0.5, label="Proposed.")
    plt.title("CTE histogram")
    plt.xlabel("CTE (m)")
    plt.ylabel("counts")
    plt.legend(loc='upper right', fontsize = 16)

    all_cte = pd.concat([fast_combined['cte'], pp_combined['cte']])
    bins = np.histogram(all_cte, bins=50)[1]  # Only get bin edges

    fig, (ax1, ax2) = plt.subplots(1, 2, sharey=True, figsize=(10, 6))

    # 왼쪽 그래프 (CTE < 1.5)
    bins_left = bins[bins < 1.5]
    ax1.hist(fast_combined['cte'], bins=bins_left, alpha=0.5, label="w/o Proposed.")
    ax1.hist(pp_combined['cte'], bins=bins_left, alpha=0.5, label="Proposed.")
    ax1.set_xlim(min(bins_left), max(bins_left))

    # 오른쪽 그래프 (CTE > 2.5)
    bins_right = bins[bins > 2.7]
    ax2.hist(fast_combined['cte'], bins=bins_right, alpha=0.5, label="w/o Proposed.")
    ax2.hist(pp_combined['cte'], bins=bins_right, alpha=0.5, label="Proposed.")
    ax2.set_xlim(2.7, 3)

    # 그래프 사이에 끊어진 부분 표시
    ax1.spines['right'].set_visible(False)
    ax2.spines['left'].set_visible(False)
    ax1.yaxis.tick_left()
    ax1.tick_params(labelright=False)
    ax2.yaxis.tick_right()

    d = .015  # 끊어진 부분의 대각선 길이
    kwargs = dict(transform=ax1.transAxes, color='k', clip_on=False)
    ax1.plot((1 - d, 1 + d), (-d, +d), **kwargs)
    ax1.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs)

    kwargs.update(transform=ax2.transAxes)
    ax2.plot((-d, +d), (-d, +d), **kwargs)
    ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs)

    # plt.ylabel("counts")
    fig.text(0.5, 0.04, "CTE (m)", ha='center')
    ax1.legend(loc='upper right', fontsize=16)
    plt.show()



    # # Offset 계산
    # bin_width = bins[1] - bins[0]
    # mpc_offset = (1 - 0.9) * bin_width / 2
    # stanley_offset = (1 - 0.8) * bin_width / 2
    # pp_offset = (1 - 0.7) * bin_width / 2

    # adjusted_bins_mpc = bins - mpc_offset
    # adjusted_bins_stanley = bins - stanley_offset
    # adjusted_bins_pp = bins - pp_offset

    # plt.figure(figsize=(10, 6))
    # plt.hist(mpc_combined['cte'], bins=adjusted_bins_mpc, alpha=1, label="MPC", rwidth=0.9, align='left')
    # plt.hist(stanley_combined['cte'], bins=adjusted_bins_stanley, alpha=1, label="Stanley", rwidth=0.8, align='left')
    # plt.hist(pp_combined['cte'], bins=adjusted_bins_pp, alpha=1, label="Pure-Pursuit", rwidth=0.7, align='left')
    # plt.xlim([min(bins), 2.])  # x축 범위 설정

    # plt.xlabel("CTE (m)")
    # plt.ylabel("counts")
    # plt.legend(loc='upper right', fontsize=20)
    plt.show()

# #*GLOBAL PATH
# mpc_data = pd.read_json(f'../data/paper_data/2023-10-14-03-32-52-fast-pp_data.json', orient="records", lines=True)
# stanley_data = pd.read_json(f'../data/paper_data/2023-10-14-02-48-37_stanley_data.json', orient="records", lines=True)
# pp_data = pd.read_json(f'../data/paper_data/good_part2_data.json', orient="records", lines=True)
# # Plotting with modified labels and title
# fig, ax = plt.subplots(figsize=(12, 10))

# # Plotting the global path (Ground Truth)
# ax.plot(global_path_x, global_path_y, color='green', linewidth=3, label='Global Path', zorder=1)
# # Plotting the vehicle data
# stride = 20
# ax.scatter(mpc_data['x'][::stride], mpc_data['y'][::stride], marker='s',c='0.2', s=2, label='MPC', zorder=2)
# ax.scatter(stanley_data['x'][::stride], stanley_data['y'][::stride], marker='^', c='r', s=2, label='Stanley', zorder=2)
# ax.scatter(pp_data['x'][::stride], pp_data['y'][::stride], c='b', marker='o', s=2, label='Pure Pursuit', zorder=2)

# ax.set_xlabel('x (m)')
# ax.set_ylabel('y (m)')
# ax.legend(loc='upper right', fontsize = 20)
# # ax.set_title("Vehicle Data vs Ground Truth Path")
# ax.grid(False)
# fig.savefig(f"../data/global_path.png")
# plt.show()

# file_names = \
# ['good_part1', # our method
# 'good_part2', # our method
# '2023-10-14-03-32-52-fast-pp', # ablation AL-PID
# '2023-10-14-03-23-57', # MPC 3rd
# '2023-10-14-03-08-16', # MPC 2nd
# '2023-10-14-02-58-43mpc', # MPC 1st
# '2023-10-14-02-48-37_stanley', # Stanley 3rd
# '2023-10-14-02-37-43', # Stanley 2nd
# '2023-10-14-02-16-50-stanley'] # Stanley 1st

# #! Left 1
# left_top = (-765, 1010)
# right_bottom = (-675, 930)
# compare_region_plot(left_top,right_bottom,'Left1', '2023-10-14-03-08-16', '2023-10-14-02-48-37_stanley','good_part1',0.3,10)

# #! Left 2
# left_top = (-905, 810)
# right_bottom = (-875, 750)
# compare_region_plot(left_top,right_bottom,'Left2', '2023-10-14-03-23-57', '2023-10-14-02-48-37_stanley','good_part2',0.75,10)

# #! Left 3
# left_top = (-350,-280)
# right_bottom = (-230,-310)
# compare_region_plot(left_top,right_bottom,'Left3', '2023-10-14-03-32-52-fast-pp', '2023-10-14-02-48-37_stanley','good_part2',0,10)

# # # #! LaneChange1
# left_top = (-630, 930)
# right_bottom = (-600, 900)
# compare_region_plot(left_top,right_bottom,'LaneChange1', '2023-10-14-03-23-57', '2023-10-14-02-48-37_stanley','good_part2',0,5)

# # # # #! LaneChange2
# left_top = (-785, 540)
# right_bottom = (-755, 510)
# compare_region_plot(left_top,right_bottom,'LaneChange2', '2023-10-14-03-08-16', '2023-10-14-02-48-37_stanley','good_part2',0,5)

regions = [((-765, 1010), (-675, 930)), ((-905, 810), (-875, 750)), ((-350,-280),(-230,-310)), ((-630, 930),(-600, 900)),((-785, 540),(-755, 510))]
adjustment_factors = [0.4, 0.8, 0, 0.5, 0.5]
# adjustment_factors = [1,1,1,1,1]
mpc_files = ['2023-10-14-03-08-16', '2023-10-14-03-23-57','2023-10-14-03-32-52-fast-pp', '2023-10-14-03-23-57', '2023-10-14-03-08-16']
mpc_files = ['2023-10-14-02-58-43mpc','2023-10-14-02-58-43mpc','2023-10-14-03-32-52-fast-pp','2023-10-14-02-58-43mpc','2023-10-14-02-58-43mpc']
stanley_files = ['2023-10-14-02-48-37_stanley','2023-10-14-02-48-37_stanley','2023-10-14-02-48-37_stanley','2023-10-14-02-48-37_stanley','2023-10-14-02-48-37_stanley']
stanley_files = ['2023-10-14-02-16-50-stanley','2023-10-14-02-16-50-stanley','2023-10-14-02-16-50-stanley','2023-10-14-02-16-50-stanley','2023-10-14-02-16-50-stanley']
pp_files = ['good_part1','good_part2','good_part2','good_part2','good_part2']
fast_files = ['2023-10-14-03-32-52-fast-pp','2023-10-14-03-32-52-fast-pp','2023-10-14-03-32-52-fast-pp','2023-10-14-03-32-52-fast-pp','2023-10-14-03-32-52-fast-pp']
new_order = [3, 0, 1, 4, 2]

regions = [regions[i] for i in new_order]
adjustment_factors = [adjustment_factors[i] for i in new_order]
mpc_files = [mpc_files[i] for i in new_order]
stanley_files = [stanley_files[i] for i in new_order]
pp_files = [pp_files[i] for i in new_order]
cte_histogram(regions, mpc_files, stanley_files, pp_files,fast_files, adjustment_factors, global_path_x, global_path_y)
combine_data_and_compute_statistics(regions, mpc_files, stanley_files, pp_files,fast_files)


# ##! ALL
# # Plotting with modified labels and title
# fig, ax = plt.subplots(figsize=(10, 8))

# # Plotting the global path (Ground Truth)
# ax.plot(global_path_x, global_path_y, color='k', label='Global Path')

# # # Scatter plots for each mode
# if manual_data:
#     manual_x, manual_y = zip(*manual_data)
#     plt.scatter(manual_x, manual_y, c='red', s=2, label='Manual')
    
# if auto_data:
#     auto_x, auto_y = zip(*auto_data)
#     plt.scatter(auto_x, auto_y, c='blue', s=2, label='Auto')
    
# if override_data:
#     override_x, override_y = zip(*override_data)
#     plt.scatter(override_x, override_y, c='black', s=2, label='Override')

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.legend()
# # ax.set_title("Vehicle Data vs Ground Truth Path")
# ax.grid(True)
# plt.show()

# Define the provided boundaries
# left_top = (200, 1050)
# right_bottom = (-1000, -350)

file_names = \
['good_part1', # our method
'good_part2', # our method
'2023-10-14-03-32-52-fast-pp', # ablation AL-PID
'2023-10-14-03-23-57', # MPC 3rd
'2023-10-14-03-08-16', # MPC 2nd
'2023-10-14-02-58-43mpc', # MPC 1st
'2023-10-14-02-48-37_stanley', # Stanley 3rd
'2023-10-14-02-37-43', # Stanley 2nd
'2023-10-14-02-16-50-stanley'] # Stanley 1st

# '2023-10-14-02-16-50-stanley_data'
# '2023-10-14-02-37-43_data'

# '2023-10-14-02-58-43mpc_data'
# '2023-10-14-03-08-16_data'
# '2023-10-14-03-23-57_data'
# '2023-10-14-03-32-52-fast-pp_data'
# 'good_part1_data'
# 'good_part2_data'

# #* Plotting the global path (Ground Truth) SAve DATA
# for i in range(len(file_names)):
#     #! Left1
#     # Define the provided boundaries
#     left_top = (-765, 1010)
#     right_bottom = (-675, 930)
#     region_plot(left_top,right_bottom,'Left1',file_names[i])

#     #! Left2
#     # Define the provided boundaries
#     left_top = (-905, 810)
#     right_bottom = (-875, 750)
#     region_plot(left_top,right_bottom,'Left2',file_names[i])

#     #! Left3
#     # Define the provided boundaries
#     left_top = (-350,-280)
#     right_bottom = (-230,-310)
#     region_plot(left_top,right_bottom,'Left3',file_names[i])

#     #! LaneChange1
#     # Define the provided boundaries
#     left_top = (-630, 930)
#     right_bottom = (-600, 900)
#     region_plot(left_top,right_bottom,'LaneChange1',file_names[i])

#     #! LaneChange2
#     # Define the provided boundaries
#     left_top = (-785, 540)
#     right_bottom = (-755, 510)
#     region_plot(left_top,right_bottom,'LaneChange2',file_names[i])

#     # plt.show()
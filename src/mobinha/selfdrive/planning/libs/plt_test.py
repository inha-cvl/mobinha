import matplotlib.pyplot as plt
import json

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

precision = 0.5
# # Load the JSON data from the provided file
with open("/home/jang/Desktop/waypoints_data_all_smooth5.json", "r") as file:
    loaded_data = json.load(file)

# Extracting data from loaded JSON
non_intp_path = loaded_data['non_intp_path']
global_path_loaded = loaded_data['global_path']
non_intp_id_loaded = loaded_data['non_intp_id']
global_id_loaded = loaded_data['global_id']

def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points
from scipy.interpolate import UnivariateSpline

def ref_interpolate_2d(points, precision, smoothing=0):
    points = filter_same_points(points)
    wx, wy = zip(*points)

    # Create a cumulative distance array
    dist = [0.0]
    for i in range(1, len(wx)):
        dist.append(dist[-1] + np.sqrt((wx[i] - wx[i-1])**2 + (wy[i] - wy[i-1])**2))
    total_distance = dist[-1]

    # Use 2nd order (quadratic) UnivariateSpline for interpolationref_interpolate_2d
    sx = UnivariateSpline(dist, wx, k=2, s=smoothing)
    sy = UnivariateSpline(dist, wy, k=2, s=smoothing)

    # Generate interpolated points
    itp_points = []
    for d in np.arange(0, total_distance, precision):
        itp_points.append((float(sx(d)), float(sy(d))))

    return itp_points, total_distance

global_path, last_s = ref_interpolate_2d(non_intp_path, precision)
global a
a = 0

from scipy.spatial.distance import euclidean

segment_length = 10
def segment_path(points, segment_length=segment_length):
    """
    Segment the given path into smaller segments.
    Each segment will have a length of `segment_length`.
    Args:
    - points: List of points [(x1, y1), (x2, y2), ...]
    - segment_length: Length of each segment
    Returns:
    - List of segments, where each segment is a list of points
    """
    segments = []
    for i in range(0, len(points), segment_length-1):
        segments.append(points[i:i+segment_length])
    return segments

def check_if_curve(segment):
    """
    Check if a segment forms a curve or a straight line.
    Args:
    - segment: List of points forming the segment
    Returns:
    - True if segment forms a curve, False otherwise
    """
    # If segment has less than 3 points, consider it as straight line
    if len(segment) < segment_length:
        return False
    
    # Calculate direction vectors between consecutive points
    vec1 = np.array(segment[1]) - np.array(segment[0])
    vec2 = np.array(segment[2]) - np.array(segment[1])
    
    # Calculate angle between direction vectors
    cos_angle = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
    
    # If angle is close to 0 or 180 degrees, segment is a straight line
    if np.isclose(cos_angle, 1) or np.isclose(cos_angle, -1):
        return False
    return True

def ref_interpolate_2d_adaptive(points, precision, curve_smoothing, straight_smoothing):
    """
    Interpolate 2D path with adaptive smoothing.
    The function interpolates the given path with different smoothing parameters for curved and straight segments.
    Args:
    - points: List of points [(x1, y1), (x2, y2), ...]
    - precision: Distance between interpolated points
    - curve_smoothing: Smoothing factor for curved segments
    - straight_smoothing: Smoothing factor for straight segments
    Returns:
    - List of interpolated points
    """
    points = filter_same_points(points)
    segments = segment_path(points)
    all_interpolated_points = []

    for segment in segments:
        # If segment has less than 3 points, add the segment points directly
        if len(segment) < segment_length:
            all_interpolated_points.extend(segment)
            continue

        wx, wy = zip(*segment)
        dist = [0.0]
        for i in range(1, len(wx)):
            dist.append(dist[-1] + np.sqrt((wx[i] - wx[i-1])**2 + (wy[i] - wy[i-1])**2))
        total_distance = dist[-1]

        # Determine if segment is a curve or straight line
        is_curve = check_if_curve(segment)
        # if is_curve:
        #     global a
        #     a = a+1
        #     print(a)
        smoothing = curve_smoothing if is_curve else straight_smoothing
        
        # Interpolation
        sx = UnivariateSpline(dist, wx, k=2, s=smoothing)
        sy = UnivariateSpline(dist, wy, k=2, s=smoothing)

        # Generate interpolated points
        segment_interpolated_points = []
        for d in np.arange(0, total_distance, precision):
            segment_interpolated_points.append((float(sx(d)), float(sy(d))))

        all_interpolated_points.extend(segment_interpolated_points)

    return all_interpolated_points

# Interpolate the entire path using the adaptive function

# Segment the path
# segments = segment_path(non_intp_path)

# Interpolate each segment with different smoothing parameters
curve_smoothing = 100
straight_smoothing = 0.01
interpolated_path_adaptive = ref_interpolate_2d_adaptive(non_intp_path, precision, curve_smoothing, straight_smoothing)

def calculate_angles(points):
    """Calculate angles (in radians) between consecutive points."""
    angles = []
    for i in range(1, len(points) - 1):
        dx1 = points[i][0] - points[i-1][0]
        dy1 = points[i][1] - points[i-1][1]
        
        dx2 = points[i+1][0] - points[i][0]
        dy2 = points[i+1][1] - points[i][1]
        
        angle1 = np.arctan2(dy1, dx1)
        angle2 = np.arctan2(dy2, dx2)
        
        angle_diff = np.abs(angle2 - angle1)
        angles.append(angle_diff)
        
    # Add dummy values at the start and end
    angles = [0] + angles + [0]
    return angles


def ref_interpolate_2d_v3(points, precision, curve_smoothing, straight_smoothing, angle_threshold=np.pi/6):
    """Interpolate a 2D path with different smoothing parameters for straight and curved segments."""
    points = filter_same_points(points)
    wx, wy = zip(*points)
    
    # Calculate angles between consecutive points
    angles = calculate_angles(points)
    
    # Create a cumulative distance array
    dist = [0.0]
    for i in range(1, len(wx)):
        dist.append(dist[-1] + np.sqrt((wx[i] - wx[i-1])**2 + (wy[i] - wy[i-1])**2))
    
    # Identify segments based on angle threshold
    segments = []
    segment = [points[0]]
    for i in range(1, len(points)):
        segment.append(points[i])
        if angles[i] > angle_threshold:
            segments.append(segment)
            segment = [points[i]]
    if segment:
        segments.append(segment)
    
    # Interpolate each segment with appropriate smoothing
    interpolated_points = []
    for segment in segments:
        if len(segment) < 3:
            # Too few points for interpolation, just add them directly
            interpolated_points.extend(segment)
            continue
        
        segment_wx, segment_wy = zip(*segment)
        segment_dist = [0.0]
        for i in range(1, len(segment_wx)):
            segment_dist.append(segment_dist[-1] + np.sqrt((segment_wx[i] - segment_wx[i-1])**2 + 
                                                           (segment_wy[i] - segment_wy[i-1])**2))
        
        # Choose smoothing based on segment type
        if max(calculate_angles(segment)[1:-1]) > angle_threshold:
            s = curve_smoothing
        else:
            s = straight_smoothing
        
        # Interpolate segment
        sx = UnivariateSpline(segment_dist, segment_wx, k=2, s=s)
        sy = UnivariateSpline(segment_dist, segment_wy, k=2, s=s)
        for d in np.arange(0, segment_dist[-1], precision):
            interpolated_points.append((float(sx(d)), float(sy(d))))
    
    return interpolated_points, dist[-1]


# Using the function to interpolate the test_wps
interpolated_path_v3, s = ref_interpolate_2d_v3(non_intp_path, precision, curve_smoothing=200, straight_smoothing=0)


from scipy.ndimage import gaussian_filter1d

def gaussian_smoothing_2d(points, sigma):
    """
    Apply Gaussian smoothing to a 2D path.
    Args:
    - points: List of points [(x1, y1), (x2, y2), ...]
    - sigma: Standard deviation for Gaussian kernel
    Returns:
    - List of smoothed points
    """
    wx, wy = zip(*points)
    smoothed_wx = gaussian_filter1d(wx, sigma=sigma)
    smoothed_wy = gaussian_filter1d(wy, sigma=sigma)
    
    return list(zip(smoothed_wx, smoothed_wy))

sigma = 10  # Standard deviation for Gaussian kernel
smoothed_path = gaussian_smoothing_2d(interpolated_path_v3, sigma)

# Plotting non_intp_path and global_path
plt.figure(figsize=(12, 12))
plt.plot([point[0] for point in non_intp_path], [point[1] for point in non_intp_path], 
         '-o', markersize=4, label="non_intp_path", alpha=0.5, color='blue')
plt.plot([point[0] for point in interpolated_path_v3], [point[1] for point in interpolated_path_v3], 
         '-o', markersize=4, label="global_path", alpha=0.2, color='red')
# plt.plot([point[0] for point in global_path], [point[1] for point in global_path], 
#          '-o', markersize=4, label="global_path", alpha=0.2, color='red')
plt.plot([point[0] for point in smoothed_path], [point[1] for point in smoothed_path], 
         '-o', markersize=4, label="global_path", alpha=0.1, color='green')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('non_intp_path vs global_path')
plt.legend()
plt.grid(True)
plt.show()
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians

def plot_rotation(yawRate, point, position):
    # Calculate the rotation matrix based on yawRate
    rotation_matrix = np.array([
        [cos(-radians(yawRate)), -sin(-radians(yawRate))],
        [sin(-radians(yawRate)), cos(-radians(yawRate))]
    ])
    
    # Calculate the difference between the point and the current position
    diff = np.array([point[0] - position[0], point[1] - position[1]])
    
    # Apply the rotation matrix to the difference vector
    rotated_diff = rotation_matrix.dot(diff)
    
    # Calculate the new point position after rotation
    rotated_point = position + rotated_diff
    
    # Plotting the original and rotated points
    plt.figure(figsize=(10, 10))
    
    # Plot the vehicle's position
    plt.plot(position[0], position[1], 'bo', label='Vehicle Position')
    
    # Plot the original point
    plt.plot(point[0], point[1], 'ro', label='Original Point')
    
    # Plot the rotated point
    plt.plot(rotated_point[0], rotated_point[1], 'go', label='Rotated Point')
    
    # Draw lines for better visualization
    plt.plot([position[0], point[0]], [position[1], point[1]], 'r--', label='Original Vector')
    plt.plot([position[0], rotated_point[0]], [position[1], rotated_point[1]], 'g--', label='Rotated Vector')
    
    # Set plot details
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'Rotation by yawRate = {yawRate} degrees')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Example usage
yawRate = 30  # degrees
point = [10, 0]
position = [0, 0]

plot_rotation(yawRate, point, position)

import rospy
from std_msgs.msg import Float32
from numpy.linalg import norm
from math import sin, cos, atan2, radians, degrees
import numpy as np

class PurePursuit:
    KPH_TO_MPS = 1 / 3.6

    def __init__(self, path):
        print("Pure Pursuit model initiated!")
        self.L = 3

    def run(self, vEgo, path, position, yawRate, cte):
        """
        Calculate the steering angle using the pure pursuit algorithm.
        
        Parameters:
        vEgo (float): Current speed of the vehicle in km/h.
        path (list of tuples): List of waypoints that the vehicle needs to follow.
        position (tuple): Current position of the vehicle (x, y).
        yawRate (float): Current yaw rate of the vehicle in degrees.
        cte (float): Cross track error, the distance from the vehicle to the path.
        
        Returns:
        float: Calculated steering angle in degrees.
        tuple: Coordinates of the target point (lookahead point).
        """
        # Calculate the lookahead distance based on current speed
        lfd = 5 + 1 * vEgo / 3.6  # Lookahead distance is proportional to the speed
        lfd = np.clip(lfd, 4, 60)  # Clip the lookahead distance to a range [4, 60] meters
        steering_angle = 0.  # Initialize the steering angle to zero
        lx, ly = path[0]  # Initialize the target point to the first point in the path
        
        for point in path:
            # Calculate the difference between the point and the current position
            diff = np.asarray((point[0] - position[0], point[1] - position[1]))
            # Create the rotation matrix to transform the difference into the vehicle's coordinate frame
            rotation_matrix = np.array(
                ((np.cos(-radians(yawRate)), -np.sin(-radians(yawRate))), 
                 (np.sin(-radians(yawRate)), np.cos(-radians(yawRate)))))
            # Rotate the difference vector to align with the vehicle's heading
            rotated_diff = rotation_matrix.dot(diff)
            if rotated_diff[0] > 0:  # Consider only the points in front of the vehicle
                dis = np.linalg.norm(rotated_diff - np.array([0, 0]))  # Calculate the distance to the point
                if dis >= lfd:  # Check if the point is beyond the lookahead distance
                    theta = np.arctan2(rotated_diff[1], rotated_diff[0])  # Calculate the angle to the target point
                    steering_angle = np.arctan2(2 * self.L * np.sin(theta), lfd)
                    # steering_angle = steering_angle + np.arctan2(0.1 * cte, vEgo) if vEgo > 6 else steering_angle
                    lx = point[0]  # Update the target point's x-coordinate
                    ly = point[1]  # Update the target point's y-coordinate
                    break
        return degrees(steering_angle), (lx, ly)  # Return the steering angle in degrees and the target point
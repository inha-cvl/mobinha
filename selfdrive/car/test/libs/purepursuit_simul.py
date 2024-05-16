import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PurePursuit:
    def __init__(self, lookahead_distance, waypoints):
        self.lookahead_distance = lookahead_distance
        self.waypoints = waypoints
        self.current_waypoint_index = 0

    def find_target_point(self, position):
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            distance = np.linalg.norm(self.waypoints[i] - position)
            if distance >= self.lookahead_distance:
                self.current_waypoint_index = i
                return self.waypoints[i]
        return self.waypoints[-1]

    def compute_steering(self, position, heading):
        target_point = self.find_target_point(position)
        dx = target_point[0] - position[0]
        dy = target_point[1] - position[1]
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = angle_to_target - heading
        return angle_diff, target_point

def simulate_pure_pursuit(initial_position, initial_heading, lookahead_distance, waypoints, speed, dt):
    pp = PurePursuit(lookahead_distance, waypoints)
    position = np.array(initial_position)
    heading = initial_heading
    trajectory = [position]

    while np.linalg.norm(position - waypoints[-1]) >= lookahead_distance:
        steering_angle, target_point = pp.compute_steering(position, heading)
        heading += steering_angle * dt
        position[0] += speed * np.cos(heading) * dt
        position[1] += speed * np.sin(heading) * dt
        trajectory.append(position.copy())
    
    return np.array(trajectory), pp

# Define parameters
lookahead_distance = 2.0
waypoints = np.array([[0, 0], [5, 5], [10, 0], [15, 5], [20, 0]])
initial_position = [0, 0]
initial_heading = 0.0
speed = 1.0
dt = 0.1

# Simulate pure pursuit
trajectory, pp = simulate_pure_pursuit(initial_position, initial_heading, lookahead_distance, waypoints, speed, dt)

# Plotting and Animation
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoints')
trajectory_line, = ax.plot([], [], 'b-', label='Trajectory')
vehicle_dot, = ax.plot([], [], 'bo')
target_dot, = ax.plot([], [], 'go', label='Target Point')
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Pure Pursuit Path Tracking')
ax.grid(True)
ax.axis('equal')

def init():
    trajectory_line.set_data([], [])
    vehicle_dot.set_data([], [])
    target_dot.set_data([], [])
    return trajectory_line, vehicle_dot, target_dot

def update(frame):
    position = trajectory[frame]
    vehicle_dot.set_data(position[0], position[1])
    target_point = pp.find_target_point(position)
    target_dot.set_data(target_point[0], target_point[1])
    trajectory_line.set_data(trajectory[:frame+1, 0], trajectory[:frame+1, 1])
    return trajectory_line, vehicle_dot, target_dot

ani = animation.FuncAnimation(fig, update, frames=len(trajectory), init_func=init, blit=True, interval=100)
plt.show()

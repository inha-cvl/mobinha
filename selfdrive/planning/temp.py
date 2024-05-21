import numpy as np
from math import sqrt

class VehiclePath:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class VelocityPlanner:
    def __init__(self, car_max_speed=20):
        self.car_max_speed = car_max_speed
    
    def curvedBaseVelocity(self, global_path, point_num, plot=False):
        out_vel_plan = []
        r_list = []
        tmp = []

        # Initialize the velocity plan and curvature radius list
        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)
            r_list.append(0)

        # Loop through the path to calculate curvature
        for i in range(point_num, len(global_path.x) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = global_path.x[i + box]
                y = global_path.y[i + box]
                x_list.append([-2 * x, -2 * y, 1])
                y_list.append((-x * x) - (y * y))

            # Calculate the curvature of the road
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T
            try:
                a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
                a = a_matrix[0]
                b = a_matrix[1]
                c = a_matrix[2]
                r = sqrt(abs(a * a + b * b - c))
            except np.linalg.LinAlgError:
                r = 0.01

            tmp.append(r)
            r_list.append(r)
            # Assuming a simple relationship between curvature and velocity
            if r < 1:  # If curvature is very high (sharp turn), lower the speed
                out_vel_plan.append(10)
            elif r < 5:  # Moderate curvature
                out_vel_plan.append(15)
            else:  # Low curvature (straight road)
                out_vel_plan.append(self.car_max_speed)

        # Ensure the velocity plan has the correct length
        for i in range(len(global_path.x) - point_num, len(global_path.x)):
            out_vel_plan.append(self.car_max_speed)
            r_list.append(0)

        if plot:
            import matplotlib.pyplot as plt
            plt.figure(figsize=(10, 5))
            plt.plot(global_path.x, out_vel_plan, label="Velocity Plan")
            plt.plot(global_path.x, r_list, label="Curvature Radius")
            plt.legend()
            plt.xlabel("Path X Coordinate")
            plt.ylabel("Velocity / Curvature Radius")
            plt.title("Velocity Plan and Curvature Radius along Path")
            plt.show()

        return out_vel_plan, r_list

# Example usage
global_path = VehiclePath(
    x=np.linspace(0, 100, 500),  # Example x coordinates
    y=np.sin(np.linspace(0, 10 * np.pi, 500))  # Example y coordinates (sine wave)
)
velocity_planner = VelocityPlanner()
out_vel_plan, r_list = velocity_planner.curvedBaseVelocity(global_path, point_num=10, plot=True)

print(out_vel_plan)

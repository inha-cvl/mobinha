"""
Eyes on the Main function for usage
"""

import numpy as np
import keyboard  # Use the keyboard library to detect keyboard input
import time
import rospy
from std_msgs.msg import Float32


class Velocity_Planner:
    def __init__(self, target_speed, mu, v, mode):
        self.target_speed = target_speed
        self.mu = mu
        self.v = v
        if mode not in ['acceleration', 'deceleration']:
            raise ValueError("Mode must be 'acceleration' or 'deceleration'")
        self.mode = mode
        


    def sigmoid_logit_function(self, s):
        if s <= 0:
            return 0
        elif s >= 1:
            return self.target_speed
        else:
            return ((1 + ((s * (1 - self.mu)) / (self.mu * (1 - s))) ** -self.v) ** -1).real * self.target_speed

    def normalize_time(self, t, target_time):
        return t / target_time

    def get_speed(self, normalized_time):
        if self.mode == 'acceleration':
            speed = self.acceleration_model(normalized_time)
        else:  # self.mode == 'deceleration'
            speed = self.deceleration_model(normalized_time)
        return speed

    def acceleration_model(self, normalized_time):
        # Use normalized time input
        speed_acceleration = np.array([self.sigmoid_logit_function(val) for val in normalized_time])
        return speed_acceleration

    def deceleration_model(self, normalized_time):
        # Use normalized time input
        speed_deceleration = np.array([self.sigmoid_logit_function(val) for val in normalized_time])
        return speed_deceleration


class Execute_velocity:
    def __init__(self, target_speed, start_margin, interval, mu, v):
        self.target_speed = target_speed
        self.start_margin = start_margin
        self.interval = interval
        self.mu = mu
        self.v = v

        self.mode = 'acceleration'
        self.pub = rospy.Publisher('/target_velocity_normal_state', Float32, queue_size=1) 

    def select_state(self):
        # Select mode
        print("Press 's' to start scheduling, 'q' to quit.")
        while True:
            # if keyboard.is_pressed('s'):
            if 1:
                print("Press 'a' for acceleration mode or 'd' for deceleration mode, 'q' to quit.")
                while True:
                    # if keyboard.is_pressed('a'):
                    if 1:
                        self.mode = 'acceleration'
                        print("Selected mode: acceleration")
                        return True
                    elif keyboard.is_pressed('d'):
                        self.mode = 'deceleration'
                        print("Selected mode: deceleration")
                        return True

    def scheduling_run(self):
        planner = Velocity_Planner(self.target_speed, self.mu, self.v, self.mode)
        elapsed_time = 0

        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time()-start_time < start_margin:
                target_v = Float32(0)
            else:
                normalized_time = planner.normalize_time(np.array([elapsed_time]), self.target_time)
                if self.mode == 'acceleration':
                    current_speed = planner.get_speed(normalized_time)
                else:
                    current_speed = planner.get_speed(1 - normalized_time)
                # print(f"Time: {elapsed_time:.2f} s, Speed: {current_speed[0]:.2f} m/s")
                target_v = Float32(current_speed[0])

            self.pub.publish(target_v)
            elapsed_time += self.interval

if __name__ == '__main__':
    # Example parameters
    start_margin = 5 # Margin time for initial stabilization
    target_speed = 60  # Target speed
    mu = 0.5  # Example value for the midpoint of the sigmoid curve
    v = 1.5   # Example value for the slope of the sigmoid curve
    interval = 1 / 50  # 50 Hz update frequency

    executor = Execute_velocity(target_speed, start_margin, interval, mu, v)

    executor.scheduling_run()
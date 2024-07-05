"""
Eyes on the Main function for usage
"""

import numpy as np
import matplotlib.pyplot as plt
import keyboard  # Use the keyboard library to detect keyboard input
import time

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
    def __init__(self, target_speed, target_time, interval, plotting_enabled, mu, v):
        self.target_speed = target_speed
        self.target_time = target_time
        self.mu = mu
        self.v = v
        self.mode = 'acceleration'
        self.interval = interval
        self.running = False
        self.plotting_enabled = plotting_enabled  # Initialize plotting flag

    def select_state(self):
        # Select mode
        print("Press 's' to start scheduling, 'q' to quit.")
        while True:
            if keyboard.is_pressed('s'):
                print("Press 'a' for acceleration mode or 'd' for deceleration mode, 'q' to quit.")
                while True:
                    if keyboard.is_pressed('a'):
                        self.mode = 'acceleration'
                        print("Selected mode: acceleration")
                        return True
                    elif keyboard.is_pressed('d'):
                        self.mode = 'deceleration'
                        print("Selected mode: deceleration")
                        return True

            elif keyboard.is_pressed('q'):
                return False
            time.sleep(0.1)

    def scheduling_run(self):
        while True:
            if not self.select_state():
                break

            # Create an instance of Velocity_Planner
            planner = Velocity_Planner(self.target_speed, self.mu, self.v, self.mode)

            plt.ion()  # Turn on interactive mode
            fig, ax = plt.subplots()
            ax.set_xlim(0, self.target_time)
            ax.set_ylim(0, self.target_speed)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Speed (m/s)')
            line, = ax.plot([], [], label=f'{self.mode.capitalize()} Model')
            plt.title(f'{self.mode.capitalize()} Model Using Sigmoid Logit Function')
            plt.legend()
            plt.grid(True)

            elapsed_time = 0
            times = []
            speeds = []

            update_interval = 10  # Update the graph every 10 calculations
            update_counter = 0

            while elapsed_time <= self.target_time:
                if keyboard.is_pressed('q'):
                    self.running = False
                    plt.ioff()
                    plt.close(fig)
                    return

                normalized_time = planner.normalize_time(np.array([elapsed_time]), self.target_time)
                if self.mode == 'acceleration':
                    current_speed = planner.get_speed(normalized_time)
                else:
                    current_speed = planner.get_speed(1 - normalized_time)
                print(f"Time: {elapsed_time:.2f} s, Speed: {current_speed[0]:.2f} m/s")
                times.append(elapsed_time)
                speeds.append(current_speed[0])
                elapsed_time += self.interval

                if self.plotting_enabled and update_counter % update_interval == 0:
                    line.set_xdata(times)
                    line.set_ydata(speeds)
                    ax.draw_artist(line)
                    fig.canvas.blit(ax.bbox)
                    fig.canvas.flush_events()
                update_counter += 1

                time.sleep(self.interval)

            # Final graph update
            if self.plotting_enabled:
                line.set_xdata(times)
                line.set_ydata(speeds)
                ax.draw_artist(line)
                fig.canvas.blit(ax.bbox)
                fig.canvas.flush_events()

            plt.ioff()  # Turn off interactive mode and display the final static graph
            plt.show()

if __name__ == '__main__':
    """ 
    Usage for this code:
    1. Run the script.
    2. Press 's' to start scheduling.
    3. Select the mode:
        - Press 'a' for acceleration mode.
        - Press 'd' for deceleration mode.
    4. Press 'q' to quit at any time.
    5. Modify 'plotting_enabled' to True/False to enable/disable plotting.
    """

    # Example parameters
    target_speed = 60  # Target speed
    target_time = 10  # Target time (seconds)
    mu = 0.5  # Example value for the midpoint of the sigmoid curve
    v = 1.5   # Example value for the slope of the sigmoid curve
    interval = 1 / 50  # 50 Hz update frequency
    plotting_enabled = True  # Set to False to disable plotting

    executor = Execute_velocity(target_speed, target_time, interval, plotting_enabled, mu, v)
    executor.scheduling_run()
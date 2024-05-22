import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

PID_STEADY_STATE_ERROR = 1  # km/h
MIN_ACC = -20
MAX_ACC = 20

class VELOCITY_PLANNER:
    def __init__(self, target_velocity = 50, max_velocity = 50, current_velocity = 0) -> None:
        self.target_v = target_velocity  # Refer from system as input
        self.max_velocity = max_velocity  # TODO: Refer from system as input
        # Accel, Brake, Keep
        self.acc_state = 'Accel'
        self.scenario_age = 100  # steps
        self.update_frequency = 10  # Hz
        self.velocity = 0
        self.start_time = 0
        self.step_counter = 0  # Reset step counter
        self.current_velocity_init = current_velocity  # it should be current velocity TODO: Refer from system as input
        self.scheduled_velocity = 0
        self.scenarios_log = deque(maxlen=2)  # Create a deque with a fixed size of 2
        self.scenarios_flag = False
        self.scenarios_done = False
        self.init_scenarios = True 
        self.scenario_overtime = False 
        self.time_log = []
        self.velocity_log = []
        self.target_velocity_log = []

    def basic_velocity_planner(self):
        if self.acc_state == 'Accel':
            self.longitudinal_acc_2_velocity()

    def longitudinal_acc_scheduling(self, ref_time):
        acc = np.clip(acc, MIN_ACC, MAX_ACC)  # make acc in a range
        ### acc scheduling with ref_time or steps
        return acc
    
    def acc_to_velocity(self, acc):
        # return acc * (1 / self.update_frequency)  # instantaneous speed
    
        if self.step_counter <= self.scenario_age / 2:
            return ((self.target_v - self.current_velocity_init) * 2) / 3
        if self.step_counter > self.scenario_age / 2:
            return (self.target_v - self.current_velocity_init)

    def velocity_scheduling(self):
        # acc = self.longitudinal_acc_scheduling(self.step_counter) ## giveup for scheduling
        # self.scheduled_velocity += self.acc_to_velocity(acc) ## for acc velocity calculation
        
        self.scheduled_velocity = self.current_velocity_init + self.acc_to_velocity(0)
        self.scheduled_velocity = min(self.scheduled_velocity, self.max_velocity)
        self.scheduled_velocity = max(self.scheduled_velocity, 0)  # Ensure velocity doesn't go negative
        self.step_counter += 1  # Increment step counter

    def scenarios_scheduling(self):
        if self.current_velocity_init > self.target_v + PID_STEADY_STATE_ERROR:
            self.acc_state = 'Brake'
            self.scenarios_flag = True
            self.scenario_age = 20  # steps

        elif self.current_velocity_init < self.target_v - PID_STEADY_STATE_ERROR:
            self.acc_state = 'Accel'
            self.scenarios_flag = True
            self.scenario_age = 20  # steps

        else:
            self.acc_state = 'Keeping'
            self.scenario_age = np.inf  # steps
            self.scenarios_flag = False

        self.scenarios_log.append(self.acc_state)
        if len(self.scenarios_log) == 1:
            self.init_scenarios = True
        elif self.scenarios_log[0] != self.scenarios_log[1]:
            self.init_scenarios = True

        if self.init_scenarios:
            self.init_scenarios = False
            self.step_counter = 0  # Reset step counter
            self.start_time = time.time()

        if self.scenarios_flag and not self.scenario_overtime:
            self.velocity_scheduling()  # -> current velocity

        self.velocity_log.append(self.scheduled_velocity)
        self.target_velocity_log.append(self.target_v)  # Record target velocity
        return self.scheduled_velocity
    
    def plot_velo(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_log, self.velocity_log, label='Scheduled Velocity (km/h)', color='blue')
        plt.plot(self.time_log, self.target_velocity_log, label='Target Velocity (km/h)', color='red', linestyle='dashed')
        plt.title('Scheduled and Target Velocity over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (km/h)')
        plt.grid(True)
        plt.legend()
        plt.show()

if __name__ == '__main__':
    vel_p = VELOCITY_PLANNER(target_velocity=40, max_velocity=50, current_velocity=0)
    count = 0
    start_time = time.time()
    current_v = 40
    target_v = 0
    while True:  # Simulate with 10Hz update frequency
        step_time = time.time() - start_time
        vel_p.scenarios_scheduling()
        vel_p.time_log.append(count)
        # time.sleep(1 / vel_p.update_frequency)
        count += 1
        if count > 50:
            break
        if count > 30:
            vel_p.target_v = target_v
            vel_p.current_velocity_init = current_v
    vel_p.plot_velo()

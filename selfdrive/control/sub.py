import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import threading
import time
target_v = []
current_v = []
err = []
time_t = []
time_c = []
data_lock = threading.Lock()
max_length = 100 
run_time = time.time()
tarV = 0


def plot_v():
    plt.ion()
    fig, ax = plt.subplots()
    target_line, = ax.plot(time_t, target_v, label='Target Velocity')
    current_line, = ax.plot(time_c, current_v, label='Current Velocity')
    error_line, = ax.plot(time_c, err, label='Error')
    plt.legend(loc='upper left')
    while not rospy.is_shutdown():
        with data_lock:
            target_line.set_ydata(target_v)
            target_line.set_xdata(time_c)
            current_line.set_ydata(current_v)
            current_line.set_xdata(time_c)
            error_line.set_ydata(err)
            error_line.set_xdata(time_c)
            ax.relim()
            ax.autoscale_view()
        plt.grid(True)
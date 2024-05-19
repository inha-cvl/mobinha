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
        plt.draw()
        plt.pause(0.05)

    plt.close(fig)

def tarV_cb(msg):
    global tarV
    tarV = msg.data

def curV_cb(msg):
    with data_lock:
        time_c.append(time.time()-run_time)
        current_v.append(msg.data)
        target_v.append(tarV)
        err.append(abs(tarV-0.7))
        for arr in [current_v, target_v, time_c, err]:
            if len(arr) > max_length:
                arr.pop(0)

def listener():
    rospy.init_node('vreceiver', anonymous=True)
    rospy.Subscriber('/mobinha/planning/target_v', Float32, tarV_cb)
    rospy.Subscriber('/mobinha/car/velocity', Float32, curV_cb)

    plot_thread = threading.Thread(target=plot_v)
    plot_thread.start()

    rospy.spin()

if __name__ == '__main__':
    listener()
import rospy
from std_msgs.msg import Int8
import tkinter as tk
import threading
import time

# 신호 발행 중지 플래그
stop_signal = threading.Event()

def publish_mode(mode):
    mode_publisher.publish(mode)
    # destination_state(mode)
    # update_button_state(mode)

# def enable_mode():
#     publish_mode(1)

# def disable_mode():
#     publish_mode(0)

def publish_reset():
    global stop_signal # 모든 signal 발행 중지 이벤트
    stop_signal.set()
    publish_signal(0)
    publish_mode(0)
    dest1_button.config(state=tk.NORMAL)
    dest2_button.config(state=tk.NORMAL)
    dest3_button.config(state=tk.NORMAL)

def repeat_publish_signal(signal):
    while not stop_signal.is_set():  # stop_signal 이벤트가 설정될 때까지 계속해서 signal 발행
        publish_signal(signal)
        time.sleep(1) 

def publish_signal(signal):
    signal_publisher.publish(signal)

def publish_dest1():
    global stop_signal
    stop_signal.set() 
    stop_signal.clear()
    threading.Thread(target=repeat_publish_signal, args=(1,)).start()
    # publish_signal(1)
    publish_mode(1)
    dest1_button.config(state=tk.DISABLED)
    dest2_button.config(state=tk.NORMAL)
    dest3_button.config(state=tk.NORMAL)
    update_button_states(1)
    time.sleep(1) 

def publish_dest2():
    global stop_signal
    stop_signal.set()
    stop_signal.clear()  
    threading.Thread(target=repeat_publish_signal, args=(2,)).start()
    # publish_signal(2)
    publish_mode(1)
    dest2_button.config(state=tk.DISABLED)
    dest1_button.config(state=tk.NORMAL)
    dest3_button.config(state=tk.NORMAL)
    update_button_states(2)
    time.sleep(1)  

def publish_dest3():
    global stop_signal
    stop_signal.set()  
    stop_signal.clear()
    threading.Thread(target=repeat_publish_signal, args=(3,)).start()
    # publish_signal(3)
    publish_mode(1)
    dest3_button.config(state=tk.DISABLED)
    dest1_button.config(state=tk.NORMAL)
    dest2_button.config(state=tk.NORMAL)
    update_button_states(3)
    time.sleep(1)  

def update_button_states(active_dest):
    dest1_button.config(state=tk.DISABLED if active_dest == 1 else tk.NORMAL)
    dest2_button.config(state=tk.DISABLED if active_dest == 2 else tk.NORMAL)
    dest3_button.config(state=tk.DISABLED if active_dest == 3 else tk.NORMAL)
    
# def update_button_state(current_mode):
#     enable_button.config(state=tk.NORMAL if current_mode == 0 else tk.DISABLED)
#     disable_button.config(state=tk.NORMAL if current_mode == 1 else tk.DISABLED)

def shutdown_hook():
    window.quit()

if __name__ == "__main__":
    rospy.init_node("ui_node")

    mode_publisher = rospy.Publisher("/mode", Int8, queue_size=10)
    signal_publisher = rospy.Publisher("/hlv_signal", Int8, queue_size=10)

    window = tk.Tk()
    window.title("Destination UI")

    button_font = ("Helvetica", 14)

    # enable_button = tk.Button(window, text="Enable", command=enable_mode, bg='#00e096', width=10, height=3, font=button_font)
    # enable_button.grid(row=0, column=0, padx=5, pady=5)

    # disable_button = tk.Button(window, text="Disable", command=disable_mode, bg="#e0004f", width=10, height=3, font=button_font)
    # disable_button.grid(row=0, column=1, padx=5, pady=5)

    reset_dest_button = tk.Button(window, text="Reset", command=publish_reset, bg="#00e096", width=10, height=3, font=button_font)
    reset_dest_button.grid(row=1, column=1, padx=5, pady=5)

    dest1_button = tk.Button(window, text="Destination1", command=publish_dest1, bg="#3446eb", width=10, height=3, font=button_font)
    dest1_button.grid(row=0, column=0, padx=5, pady=5)

    dest2_button = tk.Button(window, text="Destination2", command=publish_dest2, bg="#dbdbdb", width=10, height=3, font=button_font)
    dest2_button.grid(row=0, column=1, padx=5, pady=5)

    dest3_button = tk.Button(window, text="Destination3", command=publish_dest3, bg="#ff5340", width=10, height=3, font=button_font)
    dest3_button.grid(row=0, column=2, padx=5, pady=5)

    rospy.on_shutdown(shutdown_hook) 

    window.mainloop()
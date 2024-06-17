import rospy
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import threading
import time

# 신호 발행 중지 플래그
stop_signal = threading.Event()

#call pos1 = 461.200 1201.703 -> 37.240049888117284 126.77432268910398
#pos2 = 478.961 1365.083 -> 37.24152199920558 126.77452295955482
#pos3 = 462.726 1539.518 -> 37.24309373705744 126.77434009669331

#call_positions = [(37.240049888117284, 126.77432268910398), (37.24152199920558, 126.77452295955482), (37.24309373705744, 126.77434009669331)]
call_positions = [(35.64764980,128.40043281), (35.64865217,128.39943801), (35.64841854,128.39903501)]

#dest1 = 494.123 1500.206
#dest2 = 401.755 1542.089
#dest3 = 351.252 1118.480

#dest_positions = [(37.24273950401953,  126.7746939273392), (37.243116931069174, 126.77365293061433), (37.23930006289939, 126.77308354194406)] 
dest_positions = [(35.6477372,128.3995581), (35.64925367,128.39812751), (35.6468156,128.4013399)]

def publish(array):
    for i in range(10):  # stop_signal 이벤트가 설정될 때까지 계속해서 signal 발행
        float32multiarray = Float32MultiArray()
        float32multiarray.data = array
        gosilsil_publisher.publish(float32multiarray)
        time.sleep(0.02) 

def call_cb():
    call_pos = 1
    array = [1, call_positions[call_pos][0], call_positions[call_pos][1]]
    publish(array)

def go_cb():
    array = [5,0,0]
    publish(array)

def dest1_cb():
    array = [2, dest_positions[0][0], dest_positions[0][1]]
    publish(array)

def dest2_cb():
    array = [3, dest_positions[1][0], dest_positions[1][1]]
    publish(array)

def dest3_cb():
    array = [4, dest_positions[2][0],  dest_positions[2][1]]
    publish(array)

def shutdown_hook():
    window.quit()

if __name__ == "__main__":
    rospy.init_node("ui_node")

    gosilsil_publisher = rospy.Publisher("/gosilsil", Float32MultiArray, queue_size=1)

    window = tk.Tk()
    window.title("Destination UI")

    button_font = ("Helvetica", 14)


    call_button = tk.Button(window, text="Call", command=call_cb, bg='#ff5340', width=15, height=3, font=button_font)
    call_button.grid(row=0, column=0, padx=5, pady=5)

    go_button = tk.Button(window, text="Go", command=go_cb, bg="#3446eb", width=15, height=3, font=button_font)
    go_button.grid(row=0, column=1, padx=5, pady=5)

    dest1_button = tk.Button(window, text="Destination1", command=dest1_cb, bg="#00e096", width=10, height=3, font=button_font)
    dest1_button.grid(row=1, column=0, padx=5, pady=5)

    dest2_button = tk.Button(window, text="Destination2", command=dest2_cb, bg="#00e096", width=10, height=3, font=button_font)
    dest2_button.grid(row=1, column=1, padx=5, pady=5)

    dest3_button = tk.Button(window, text="Destination3", command=dest3_cb, bg="#00e096", width=10, height=3, font=button_font)
    dest3_button.grid(row=1, column=2, padx=5, pady=5)

    rospy.on_shutdown(shutdown_hook) 

    window.mainloop()
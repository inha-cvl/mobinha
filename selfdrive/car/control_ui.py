#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16, Bool
import tkinter as tk


class ControlUI():
    def __init__(self):
        rospy.init_node('control_UI')
        pub = rospy.Publisher('can_cmd', Int16, queue_size=1)
        self.ot = rospy.Publisher('overtake', Bool, queue_size=1)

        rate = rospy.Rate(10)

        self.mode = 0

        window = tk.Tk()
        lat_button = tk.Button(text="Latitudinal", command=self.lat_callback)
        lon1_button = tk.Button(text="Longitudinal",
                                command=self.lon1_callback)
        lon2_button = tk.Button(text="Longitudinal",
                                command=self.lon2_callback)
        disable_button = tk.Button(text="Disable", command=self.dis_callback)

        lat_button.pack()
        lon1_button.pack()
        lon2_button.pack()
        disable_button.pack()

        while not rospy.is_shutdown():
            window.update()
            pub.publish(self.mode)
            rate.sleep()

    def lat_callback(self):
        self.mode = 1 if self.mode == 0 else 0
        print(self.mode)

    def lon1_callback(self):
        self.mode = 2 if self.mode == 3 else 3
        print(self.mode)

    def lon2_callback(self):
        self.mode = 2 if self.mode == 4 else 4

    def dis_callback(self):
        self.ot.publish(True)


if __name__ == '__main__':
    ControlUI()

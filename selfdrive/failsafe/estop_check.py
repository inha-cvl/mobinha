#!/usr/bin/env python3
import sys
import signal
import serial
import threading
import rospy
from std_msgs.msg import Int8

class Activate_Signal_Interrupt_Handler:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, sig, frame):
        print('\nYou pressed Ctrl+C! Never use Ctrl+Z!')
        sys.exit(0)

class EstopChecker:
    def __init__(self):
        # Serial Connect
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200)
        except:
            print('Try: $sudo chmod a+rw /dev/ttyUSB0')

        # ROS
        rospy.init_node('estop_checker', anonymous=False)
        self.pub = rospy.Publisher('estop', Int8, queue_size=1)
        self.msg = Int8()
            
    def run(self):
        print('====================estop start')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            packet = self.ser.readline()
            data = int(packet.decode())
            
            if data == 0:
                self.msg.data = 0
            else:
                self.msg.data = 1

            self.pub.publish(self.msg)
            rate.sleep()

if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()
    EC = EstopChecker()
    EC.run()

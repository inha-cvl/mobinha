#!/usr/bin/env python3

import rospy, time
from std_msgs.msg import Int8MultiArray, Int8, Bool
from geometry_msgs.msg import Vector3
from sbg_driver.msg import SbgEkfNav 

class ModeChanger:
    def __init__(self):
        self.mode = 0
        self.prev_mode = 0
        self.mode_set = 0
        self.tor = 0
        self.tor_cnt = 0

        rospy.Subscriber('/mode_set', Int8, self.mode_set_callback)
        rospy.Subscriber('/tor', Int8, self.tor_callback)
        self.mode_pub = rospy.Publisher("/mode", Int8, queue_size=1)
    
    def mode_set_callback(self, msg):
        print(msg.data)
        self.mode_set = msg.data

    def tor_callback(self, msg):
        self.tor = msg.data

    
    def tor_print(self, type):
        if type==0:
            print("Stable")
        elif type==1:
            print("TOR : Break Pedal")
        elif type == 2 :
            print("TOR : Accel")
        elif type == 3 :
            print("TOR : HandleToq")
        elif type == 4 :
            print("TOR : E-Stop")
        elif type == 5 :
            print("TOR : Sensor Error")
        elif type == 6 :
            print("TOR : System Error")
        elif type == 7 :
            print("TOR : Lane Departure")
        elif type == 8 :
            print("TOR : AEB")


    def modePublisher(self):
        mode_msg = Int8()

        # TOR8 = AEB : Do Not Need to Change Mode
        if self.prev_mode == 1 and self.tor != 0 and self.tor != 8: # for kiapi and self.tor != 7: 
            if self.tor_cnt >= 10:
                mode_msg.data = 0
                self.mode_set = 0
                self.tor_print(self.tor)
                self.tor_cnt = 0
            else:
                mode_msg.data = 1
                self.tor_cnt += 1

        elif self.mode_set == 1:
            mode_msg.data = 1
            print(self.mode_set, "AUTOPILOT")

        elif self.mode_set == 0:
            mode_msg.data = 0
            print(self.mode_set, "MANUAL")
        else:
            print("Else Case")
        self.prev_mode = int(mode_msg.data)


        self.mode_pub.publish(mode_msg)


def main():
    rospy.init_node('ModeChanger')
    chgmod = ModeChanger()
    rate = rospy.Rate(10)
    print("Ready to mode switch.")
    while not rospy.is_shutdown():
        chgmod.modePublisher()
        rate.sleep()
    
    rospy.spin()
    

if __name__ == '__main__':
    main()

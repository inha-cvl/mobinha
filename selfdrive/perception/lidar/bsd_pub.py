#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8MultiArray
from jsk_recognition_msgs.msg import BoundingBoxArray

class BSD:
    def __init__(self):
        self.car_array = []

        rospy.Subscriber('/pillar_box_array', BoundingBoxArray, self.pillar_box_callback)

        self.bsd_check_pub = rospy.Publisher("/BSD_check", Int8MultiArray, queue_size=1)
        
    def pillar_box_callback(self, msg):
        self.car_array = msg.boxes  

    def bsdPublisher(self):
        bsd_msg = Int8MultiArray()
        bsd_msg.data = [0,0]
        car_x_array = []
        car_y_array = []
        for box in self.car_array:
            car_x = box.pose.position.x
            car_y = box.pose.position.y
            car_x_array.append(car_x)
            car_y_array.append(car_y)
            if any(-30< x for x in car_x_array) & any(30> x for x in car_x_array) & any(2 < y for y in car_y_array) & any(5 > y for y in car_y_array):
                bsd_msg.data[0] = 1
            else:
                bsd_msg.data[0] = 0

            if any(-30< x for x in car_x_array) & any(30> x for x in car_x_array) & any(-5 < y for y in car_y_array) & any(-2 > y for y in car_y_array):
                bsd_msg.data[1] = 1
            else:
                bsd_msg.data[1] = 0

        self.bsd_check_pub.publish(bsd_msg)


def main():
    rospy.init_node('BSD')
    b = BSD()
    rate = rospy.Rate(11)
    print("Ready to BSD CHECK.")
    while not rospy.is_shutdown():
        b.bsdPublisher()
        rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    main()

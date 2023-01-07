#!/usr/bin/env python
# -*- coding: utf-8 -*-
  
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Imu

class IMUParser:
    def __init__(self):
        rospy.init_node('imu', anonymous=True)    
        self.image_sub = rospy.Subscriber("/imu", Imu, self.callback)
        rospy.spin()

    def callback(self, data):
        rate=rospy.Rate(5)
        rate.sleep()
        print("orientation:")
        print("x : {} y : {} z : {} w : {}".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        print("\nangular_velocity:")
        print("x : {} y : {} z : {}".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
        print("\nlinear_acceleration:")
        print("x : {} y : {} z : {}".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
        print("\n===============================================================\n")
      

if __name__ == '__main__':
    try:
        imu_parser = IMUParser()
    except rospy.ROSInterruptException:
        pass
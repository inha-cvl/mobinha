#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from morai_msgs.msg import GPSMessage



class erp_gps():
    def __init__(self):
        rospy.init_node('gps', anonymous=True)

    
        #subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)

        rospy.spin()



    def gpsCB(self,data):
        print("latitude {}".format(data.latitude))
        print("longitude {}".format(data.longitude))
        print("eastOffset {}".format(data.eastOffset))
        print("northOffset {}".format(data.northOffset))

    
if __name__ == '__main__':
    try:
        gps=erp_gps()
    except rospy.ROSInterruptException:
        pass
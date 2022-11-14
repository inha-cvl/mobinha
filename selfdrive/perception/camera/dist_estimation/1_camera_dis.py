#!/usr/bin/python
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math

from std_msgs.msg import Float32MultiArray
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

import signal
import sys

class CameraObject():
    def __init__(self):
        rospy.init_node('Image_node')
        rospy.loginfo("-------subscriber_node start!-------")

        rospy.Subscriber("/Camera/Front60/od_bbox", Float32MultiArray, self.bbox_front60)
        rospy.Subscriber("/odom", Float32MultiArray, self.get_odom)
        self.pub_camera_ob_marker = rospy.Publisher('/camera_ob', Marker, queue_size=1)

        self.pose = [0,0,0,0,0]

        poi_left = [718.0,224.0,295.0,640.0]
        poi_right = [718.0,1070.0,295.0,640.0]
        poi_11 = [446.65,11.13,534.19,7.36]
        poi_21 = [393.05,21.09,446.65,11.13]
        poi_29 = [348.39,29.54,393.05,21.09]
        poi_37 = [346.6,37.25,348.39,29.54]
        poi_41 = [334.09,41.96,346.6,37.25]
        poi_50 = [330.52,50.0,334.09,41.96]
        poi_55 = [328.73,55.79,330.52,50.0]
        poi_59 = [325.16,59.14,328.73,55.79]
        self.poi_front60_all = [poi_left,poi_right,poi_11,poi_21,poi_29,poi_37,poi_41,poi_50,poi_55,poi_59]


    def line_formula(self,k,b,x):
        y = k*x+b
        return y

    def line_para(self,x1,y1,x2,y2):
        k = (y1 - y2)/(x1-x2)
        b = ((x1*y2)-(x2*y1))/(x1-x2)
        return k , b

    def point2line(self,x1,y1,x2,y2,x):
        k,b = self.line_para(x1,y1,x2,y2)
        y = self.line_formula(k,b,x)
        return y 

    def bbox_front60(self,msg):
        if msg.data is not None:
            data  = list(msg.data)                        
            distance_2 = self.distance_front(data,self.poi_front60_all)
        self.om_marker(distance_2)

    def get_odom(self,msg):
        self.pose = msg.data

    def distance_front(self,data,poi_all):
        poi_left = poi_all[0]
        poi_right = poi_all[1]
        poi_11 = poi_all[2]
        poi_21 = poi_all[3]
        poi_29 = poi_all[4]
        poi_37 = poi_all[5]
        poi_41 = poi_all[6]
        poi_50 = poi_all[7]
        poi_55 = poi_all[8]
        poi_59 = poi_all[9]
        
        count=0

        distance_2 = 100.0
        distance_temp = 100.0
        a = len(data)/5
        for l in range(int(a)):
            if data[4+l*5] == 2.0:
            # if data[4+l*5] == 2.0 or data[4+l*5] == 0:
            # if data[l*5-1] ==2:
                lt_x = data[0+l*5] * 2.0
                lt_y = data[1+l*5]  * (720.0 / 403.0)#####
                rb_x = data[2+l*5] * 2.0
                rb_y = data[3+l*5] * (720.0 / 403.0)####
                point = [(rb_x+lt_x)/2.0, rb_y, 1.0]
                if (rb_x+lt_x)/2.0 >320.0 and (rb_x+lt_x)/2.0 <960.0:
                    min_x = self.point2line(poi_left[0],poi_left[1],poi_left[2],poi_left[3],point[1])
                    max_x = self.point2line(poi_right[0],poi_right[1],poi_right[2],poi_right[3],point[1])
                    
                    # min_x = line_formula((-537.0/437.0),(428729.0/437.0),point[1])#leftx = -537/437,y = 428729/437
                    # max_x = line_formula((541.0/436.0),(31563.0/109.0),point[1])#right x = 541/436,y = 31563/109
                    if point[0]>min_x and point[0]<max_x :
                        print('point')
                        print(point)
                        if point[1]> poi_11[0]:#<11.13 , 446
                            poi = poi_11
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])
                            print('11')
                            print("distance")
                            print(distance_temp)
                        elif point[1]> poi_21[0]:#<21.09, 393.05
                            poi = poi_21
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])                       
                            print('21')
                            print("distance")
                            print(distance_temp)
                        elif point[1]> poi_29[0]:#<29.54 , 348.39
                            poi = poi_29
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])                       
                            print('29')
                            print("distance")
                            print(distance_temp)
                        elif point[1]> poi_37[0]:#<37.25 , 346.6
                            poi = poi_37
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])
                            print('37')
                            print("distance")
                            print(distance_temp)

                        elif point[1]> poi_41[0]:#<41.96 , 334.09
                            poi = poi_41
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])
                            print('41')
                            print("distance")
                            print(distance_temp)

                        elif point[1]> poi_50[0]:#<50 , 330.52
                            poi = poi_50
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])
                            print('50')
                            print("distance")
                            print(distance_temp)

                        elif point[1]> poi_55[0]:#<55.79 , 328.73
                            poi = poi_55
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])
                            print('55')
                            print("distance")
                            print(distance_temp)

                        elif point[1]> poi_59[0]:#<59.14 , 325.16
                            poi = poi_59
                            distance_temp = self.point2line(poi[0],poi[1],poi[2],poi[3],point[1])
                            print('59')
                            print("distance")
                            print(distance_temp)
                        if distance_temp < distance_2:
                            distance_2 = distance_temp
                        # if distance[1] < 10 and distance[1] > -10:
                            #       print(distance)
                        count+=1

                        return distance_2

    def om_marker(self,distance_2):
        dummy = 1
        if distance_2 == None:
            distance_2 = 0
            dummy = 0
            
        marker_ob = Marker()

        theta = math.radians(self.pose[2])

        ob_id = 0
        ##marker
        marker_ob.header.frame_id = 'world'
        marker_ob.type = marker_ob.SPHERE
        marker_ob.action = marker_ob.ADD
        marker_ob.scale.x = 1.0
        marker_ob.scale.y = 1.0
        marker_ob.scale.z = 1.0
        marker_ob.color.a = 1.0
        marker_ob.color.r = 1.0
        marker_ob.color.g = 1.0
        marker_ob.color.b = 1.0
        marker_ob.id = ob_id
        marker_ob.pose.orientation.w = 1.0
        marker_ob.pose.position.x = distance_2 * math.cos(theta) + self.pose[0] * dummy
        marker_ob.pose.position.y = distance_2 * math.sin(theta) + self.pose[1] * dummy
        marker_ob.pose.position.z = -1.0
        marker_ob.lifetime = rospy.Duration.from_sec(0.3)

        self.pub_camera_ob_marker.publish(marker_ob)
        
if __name__== '__main__':
    CameraObject()
    rospy.spin()


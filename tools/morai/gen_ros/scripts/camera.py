#!/usr/bin/env python
  
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage


class IMGParser:
    def __init__(self):
        rospy.init_node('camera', anonymous=True)
    
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        rospy.spin()

    def callback(self, data):

        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)
      

if __name__ == '__main__':
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass
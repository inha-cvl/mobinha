import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle

import time
from std_msgs.msg import Float32MultiArray
import rospy
import rosbag
import numpy as np
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray

def callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img


class Point(object):
    x = 0
    y = 0
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
 
class Line(object):
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
 
class Object_msg:
    def __init__(self):
        self.pub_camera_ob_marker = rospy.Publisher('/camera_ob_marker', MarkerArray, queue_size=1)

def GetLinePara(line):
    line.a = line.p1.y - line.p2.y
    line.b = line.p2.x - line.p1.x
    line.c = line.p1.x * line.p2.y - line.p2.x * line.p1.y

def undistort(img, camera_matrix, dist_coeffs):
    DIM = (img.shape[1], img.shape[0])
    h,w = img.shape[:2]
    # map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, dist_coeffs, np.eye(3), camera_matrix, DIM, cv2.CV_16SC2)
    # undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # undistorted_img = img.copy()
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, camera_matrix)
    return undistorted_img

def read_txt(path, name ):
    cam_param = []
    with open(path + name + '.txt', 'r') as f:
        f = f.read()
        f = f.replace('[','').replace(']','').replace('\n',',')
        for i in f.split(','):
            if i != '':
                cam_param.append(float(i))
    return cam_param
 
def GetCrossPoint(l1,l2):
    GetLinePara(l1)
    GetLinePara(l2)
    d = l1.a * l2.b - l2.a * l1.b
    p = Point()
    p.x = (l1.b * l2.c - l2.b * l1.c)*1.0 / d
    p.y = (l1.c * l2.a - l2.c * l1.a)*1.0 / d
    return p

def line_formula(k,b,x):
    y = k*x+b
    return y

def bbox(msg):
    src = np.float32([
      [9,344],#269
      [19,661],#125.5
      [1263,662],#508
      [1270,377]
                     ])#370
    dst = np.float32([
      [44.6676,22.4937],
      [5.76172,2.54677],
      [5.67765,-2.48574],
      [43.4746,-21.5407]
                     ])
    # from 3D to 2D 
    m = cv2.getPerspectiveTransform(src, dst)
    p1 = Point(100,717)
    p2 = Point(637,287)
    line1 = Line(p1,p2)

    p3 = Point(1178,716)
    p4 = Point(637,287)
    line2 = Line(p3,p4)

    # Pc = GetCrossPoint(line1,line2);
    print("-"*20)
    
    # plt.ion()
    
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # # ax = fig.add_subplot(111, projection='3d')
    
    count = 0
    distance_2 = 0

    if msg.data is not None:
        data  = list(msg.data)                        
        a = len(data)/5
        for l in range(int(a)):
            if data[4+l*5] ==2 or data[4+l*5]==0:
            # if data[l*5-1] ==2 or data[l*5-1]==0:
                lt_x = data[0+l*5] * 2
                lt_y = data[1+l*5]  * (720.0 / 403.0)
                rb_x = data[2+l*5] * 2
                rb_y = data[3+l*5] * (720.0 / 403.0)
                point = [(rb_x+lt_x)/2.0, rb_y, 1]
                if (rb_x+lt_x)/2.0 >320 and (rb_x+lt_x)/2.0 <960:
                    min_x = line_formula((-537.0/437.0),(428729.0/437.0),point[1])#leftx = -537/437,y = 428729/437
                    max_x = line_formula((541.0/436.0),(31563.0/109.0),point[1])#right x = 541/436,y = 31563/109

                    if point[0]>min_x and point[0]<max_x :
                        print(count)
                        print('point')
                        print(point)
                        # distant = m.dot(point)
                        # distance = distant/distant[2]

                        distance_2 = 0
                        if point[1]> 446.65:#<11.13
                            # distance_2 = -(27.0/22.0)* point[1] + (10073.0/22.0)
                            distance_2 = line_formula((-377.0/8754.0),(26581907.0/875400.0),point[1])
                            print('11')
                            print("distance")
                            print(distance_2)
                        elif point[1]> 393.05:#<21.09
                            distance_2 = line_formula((-249.0/1340.0),(2522601.0/26800.0),point[1])                                
                            print('21')
                            print("distance")
                            print(distance_2)
                        elif point[1]> 348.39:#<29.54
                            distance_2 = line_formula((-845.0/4466.0),(6090217.0/63800.0),point[1])
                            print('29')
                            print("distance")
                            print(distance_2)
                        elif point[1]> 346.6:#<37.25
                            distance_2 = line_formula((-771.0/179.0),(5477927.0/3580.0),point[1])
                            print('37')
                            print("distance")
                            print(distance_2)

                        elif point[1]> 334.09:#<41.96
                            distance_2 = line_formula((-157.0/417.0),(1398989.0/8340.0),point[1])
                            print('41')
                            print("distance")
                            print(distance_2)

                        elif point[1]> 330.52:#<50
                            distance_2 = line_formula((-268.0/119.0),(2363234.0/2975.0),point[1])
                            print('50')
                            print("distance")
                            print(distance_2)

                        elif point[1]> 328.73:#<55.79
                            distance_2 = line_formula((-579.0/179.0),(5008027.0/4475.0),point[1])
                            print('55')
                            print("distance")
                            print(distance_2)

                        elif point[1]> 325.16:#<59.14
                            distance_2 = line_formula((-335.0/357.0),(6502079.0/17850.0),point[1])
                            print('59')
                            print("distance")
                            print(distance_2)
                        distance_2 = distance_2
                        # if distance[1] < 10 and distance[1] > -10:
                        #       print(distance)
                        count+=1
                        d = [distance_2,point[1]]
                        array.append(d)
                        # array= np.append(d,axis=0)


                        # for i in range(len(array)):
                        #     x,y = array[:,0],array[:,1]
                        #     ax.scatter(x,y,color='hotpink')
                        #     # ax.scatter(array[0],array[1],color='hotpink')
                        #     ax.set_xlabel("Distance (m)")
                        #     ax.set_ylabel("Pixel")
    
    if distance_2 != 0:
        om = Object_msg()
        camera_ob_marker_array = MarkerArray()
        marker_ob = Marker()

        ob_id = 0
        ##marker
        # marker_ob.header.frame_id = 'ego_car'
        marker_ob.header.frame_id = 'os_sensor'
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
        marker_ob.pose.position.x = distance_2
        marker_ob.pose.position.y = 0
        marker_ob.pose.position.z = -1.0
        marker_ob.lifetime = rospy.Duration.from_sec(0.3)
        camera_ob_marker_array.markers.append(marker_ob)
        om.pub_camera_ob_marker.publish(camera_ob_marker_array)

        camera_ob_marker_array = MarkerArray()


    
def msgCallback3(msg):
   # if flag == True:
   img_np_arr = np.fromstring(msg.data, np.uint8)
   # front_190 = CvBridge().imgmsg_to_cv2(img_np_arr, desired_encoding="rgb8")
   front_190 = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
   # front_190 = cv2.resize(front_190,(640,403))
   front_190 = cv2.resize(front_190,(1280,720))

   # front_190 =undistort(front_190, camera_matrix, dist_coeffs)
   # front_190 = cv2.undistort(front_190, camera_matrix, dist_coeffs, None, camera_matrix)

   # front_190 = cv2.resize(front_190, (640, 403))
   cv2.namedWindow("front_190", 0)
   cv2.imshow("front_190", front_190)
   cv2.waitKey(1)


if __name__== '__main__':
   
    rospy.init_node('Image_node')
    rospy.loginfo("-------subscriber_node start!-------")

    # '''NIRO'''
    # path = '/home/leolix/Desktop/NGV/good_performance/NIRO/mat_inter/'
    # # front_190 = 'front_190_mat_inter'
    # front_60 = 'front_60_mat_inter'
    # # left = 'left_mat_inter'
    # # right = 'right_mat_inter'

    # name = front_60

    # cam_param = read_txt(path, name)
    # camera_matrix = np.array([[cam_param[0], cam_param[1], cam_param[2]], 
                             # [cam_param[3], cam_param[4], cam_param[5]], 
                             # [cam_param[6], cam_param[7], cam_param[8]]])
    # dist_coeffs = np.array([[cam_param[9], cam_param[10], cam_param[11], cam_param[12], cam_param[13]]])
    
    array = []
    # # array = np.array(array)




    rospy.Subscriber("/gmsl_camera/dev/video0/compressed", CompressedImage, msgCallback3)
    rospy.Subscriber("/Camera/Front60/od_bbox", Float32MultiArray, bbox)

    # print(array)

    # fig.canvas.draw()
    # fig.canvas.flush_events()
    # time.sleep(0.01)


    rospy.spin()

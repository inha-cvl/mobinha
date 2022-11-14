import torch
from torch import nn
import torch.nn.functional as F
from tool.torch_utils import *
from tool.yolo_layer import YoloLayer
from models.Yolov4_model import Yolov4

import cv2
import time
import copy
import argparse

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
#from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from cv_bridge import CvBridge, CvBridgeError

from tool.utils import *
from tool.torch_utils import *

from API.tracker import Tracker
from API.drawer import Drawer
from calibration import Calibration

#dont need
# print("good")
# exit()
##########

parser = argparse.ArgumentParser(description='Test the Model on video or make evaluation format by pascal VOC')

parser.add_argument('--weightfile', default="weight/yolov4.pth")#'/media/dong/Store2/data/2020automous/checkpoints/1020_2/Yolov4_epoch36.pth', help="Using pre_trained weight")
parser.add_argument('--n_classes', default=80, help="Number of classes")
parser.add_argument('--namesfile', default="data/coco.names", help="Label name of classes")
parser.add_argument('--gpu_num', default=0, help="Use number gpu")

args = parser.parse_args()

def callback(msg):
    # before = time.time()
    global calibration
    global cur_img
    global get_new_img_msg
    global temp_img
    global camera_matrix
    global dist_coeffs

    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    img = cv2.resize(img, (1280, 720))#, interpolation=cv2.INTER_AREA)
    img = cv2.undistort(img, camera_matrix, dist_coeffs)
    temp_img = copy.copy(img)


    cur_img['img'] = img #calibration.undistort(img)
    cur_img['header'] = msg.header
    get_new_img_msg = True
    # print("time =")
    # print((time.time()-before))

# def get_bbox_arry_msg(bboxes, label, header):
#     # bbox_array_msg = BoundingBoxArray()
#     bbox_array_msg.header = header

#     for i in bboxes:
#         tl = (i[0], i[1])
#         w = i[2] - i[0]
#         h = i[3] - i[1]
#         id = i[4]
#         label = i[-1]

#         bbox_msg = BoundingBox()
#         bbox_msg.pose.position.x = tl[0]
#         bbox_msg.pose.position.y = tl[1]
#         bbox_msg.pose.position.z = 0.0
#         bbox_msg.dimensions.x = w
#         bbox_msg.dimensions.y = h
#         bbox_msg.dimensions.z = 0.0
#         bbox_msg.value = id
#         bbox_msg.label = label

#         bbox_array_msg.boxes.append(bbox_msg)   
    
#     return bbox_array_msg
'''
def lidar_point_to_image_front60(msg):
    global temp_img
    # temp_img = cv2.resize(temp_img, (640, 480))
    x = 0
    y = 0
    # time22 = time.time()
    for num, i in enumerate(msg.data):
        if(num%2 == 0):
            x = int(i)
            continue
        else:
            y = int(i)
            cv2.circle(temp_img, (x,y), 1, (0,0,255), -1)
    # print(time.time()-time22)
	
    # temp_img = cv2.resize(temp_img, (320, 180))
    cv2.imshow("front60_img", temp_img)
    cv2.waitKey(1)
'''
'''
def lidar_point_to_image_backleft(msg):
    global temp_img
    temp_img = cv2.resize(temp_img, (1280, 720))
    x = 0
    y = 0
    # time22 = time.time()
    for num, i in enumerate(msg.data):
        if(num%2 == 0):
            x = int(i)
            continue
        else:
            y = int(i)
            cv2.circle(temp_img, (x,y), 1, (0,0,255), -1)
    # print(time.time()-time22)
	
    # temp_img = cv2.resize(temp_img, (320, 180))
    cv2.imshow("backleft_img", temp_img)
    cv2.waitKey(1)

'''
def lidar_point_to_image_backright(msg):
    global temp_img
    # temp_img = cv2.resize(temp_img, (640, 480))
    x = 0
    y = 0
    # time22 = time.time()
    for num, i in enumerate(msg.data):
        if(num%2 == 0):
            x = int(i)
            continue
        else:
            y = int(i)
            cv2.circle(temp_img, (x,y), 1, (0,0,255), -1)
    # print(time.time()-time22)
	
    # temp_img = cv2.resize(temp_img, (320, 180))
    cv2.imshow("backright_img", temp_img)
    cv2.waitKey(1)


if __name__ == "__main__":
    import sys
    import cv2

    try:
        calibration = Calibration('calibration_data/camera.txt', 'calibration_data/old_camera_lidar.txt')

        cur_img = {'img':None, 'header':None}
        temp_img = None
        get_new_img_msg = False

        rospy.init_node('object_detection')
        # rospy.Subscriber('/gmsl_camera/port_0/cam_0/image_raw/compressed', CompressedImage, callback)
        # rospy.Subscriber('/vds_node_localhost_2218/image_raw/compressed', CompressedImage, callback)

	# new
	# camera0 = left, camera1 = right, camera2 = front60, camera3 = front190
	# same	
	# gmsl_camera/dev/video0/compressed

	# old
	# cam0 = right, cam1 = front60, cam2 = front190, cam3 = left
	
	# newnew
	# 

        # front
        # rospy.Subscriber('/gmsl_camera/dev/video2/compressed', CompressedImage, callback)
        # rospy.Subscriber('/lidar_to_camera_array_front60', Float32MultiArray, lidar_point_to_image_front60)

        # left
        # rospy.Subscriber('/gmsl_camera/dev/video0/compressed', CompressedImage, callback)
        # rospy.Subscriber('/lidar_to_camera_array_backleft', Float32MultiArray, lidar_point_to_image_backleft)

        #right
        rospy.Subscriber('/gmsl_camera/port_0/cam_0/image_raw/compressed', CompressedImage, callback)
        rospy.Subscriber('/lidar_to_camera_array_backright', Float32MultiArray, lidar_point_to_image_backright)

        # rospy.Subscriber('/lidar_to_camera_array_backright', Float32MultiArray, lidar_point_to_image_backright)
        pub_od = rospy.Publisher('/od_result', Image, queue_size=1, latch=True)
        pub_traffic_light = rospy.Publisher('/traffic_light', Int32, queue_size=1, latch=True)
        
        # pub_bbox = rospy.Publisher("/od_bbox", BoundingBoxArray, queue_size=10)
        bridge = CvBridge()

        cam_param = []
        # with open('./camera.txt', 'r') as f:
        # with open('./camera_front60.txt', 'r') as f:
        # with open('./camera_left.txt', 'r') as f:
        with open('./calibration_data/camera_right.txt', 'r') as f:
            for i in f.readlines():
                for val in i.split(','):
                    cam_param.append(float(val))

        camera_matrix = np.array([[cam_param[0], cam_param[1], cam_param[2]], 
                                  [cam_param[3], cam_param[4], cam_param[5]], 
                                  [cam_param[6], cam_param[7], cam_param[8]]])
        # dist_coeffs = np.array([[cam_param[9]], [cam_param[10]], [cam_param[11]], [cam_param[12]]])
        dist_coeffs = np.array([[cam_param[9], cam_param[10], cam_param[11], cam_param[12], cam_param[13]]])

        model = Yolov4(args.weightfile, n_classes=args.n_classes, inference=True)
        pretrained_dict = torch.load(args.weightfile)
        model.load_state_dict(pretrained_dict)

        torch.cuda.set_device(args.gpu_num)
        model.cuda()
        model.eval()
        torch.backends.cudnn.benchmark = True
        print ('Current cuda device : %s'%(torch.cuda.current_device()))
        
        #class_names = load_class_names('data/songdo.names')

        interval = 1
        img_shape = (720, 1280)

        tracker = Tracker(img_shape, min_hits=1, num_classes=args.n_classes, interval=interval)  # Initialize tracker
        drawer = Drawer(args.namesfile)  # Inidialize drawer class

        moving_tra, moving_det = 0., 0.
        frame_ind = 0

        ##
        thickness = 1
        fontSize = 0.7
        fontFace = cv2.FONT_HERSHEY_TRIPLEX
        
        # pub2 = rospy.Publisher('camera_od_front60', Float32MultiArray, queue_size=20)
        # pub2 = rospy.Publisher('camera_od_backleft', Float32MultiArray, queue_size=20)
        pub2 = rospy.Publisher('camera_od_backright', Float32MultiArray, queue_size=20)
        # sign_count = [0, 0, 0, 0]
        ##
        while not rospy.is_shutdown():
            test_time = time.time()
            if get_new_img_msg:
                start = time.time()
                dets_arr, labels_arr, is_dect = None, None, None
                if np.mod(frame_ind, interval) == 0:
                    img = cv2.resize(cur_img['img'], (320, 320))
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    
                    s_time = time.time()
                    bbox = do_detect(model, img, 0.7, 0.4)[0]
                    orig_im = copy.copy(img)
                    orig_im = cv2.cvtColor(orig_im, cv2.COLOR_BGR2RGB)
                    orig_im = cv2.resize(orig_im,(1280,720))
                    
                    original_img_copy = copy.copy(orig_im)
                    if len(bbox) > 0: 
                        bbox = np.vstack(bbox)
                        output = copy.copy(bbox)

                        output[:,0] = (bbox[:,0] - bbox[:,2] / 2.0) * 1280
                        output[:,1] = (bbox[:,1] - bbox[:,3] / 2.0) * 720
                        output[:,2] = (bbox[:,0] + bbox[:,2] / 2.0) * 1280
                        output[:,3] = (bbox[:,1] + bbox[:,3] / 2.0) * 720

                        dets_arr, labels_arr = output[:,0:4], output[:,-1].astype(int)
                    else:
                        dets_arr, labels_arr = np.array([]), np.array([])

                    is_dect = True
                    
                elif np.mod(frame_ind, interval) != 0:
                    dets_arr, labels_arr = np.array([]), np.array([])
                    is_dect = False

                pt_det = (time.time() - start)

                tracker_arr = tracker.update(dets_arr, labels_arr, is_dect=is_dect)

                pt_tra = (time.time() - start)
                
                if frame_ind != 0:
                    moving_tra = (frame_ind / float(frame_ind + 1) * moving_tra) + (1. / float(frame_ind + 1) * pt_tra)
                    moving_det = (frame_ind / float(frame_ind + 1) * moving_det) + (1. / float(frame_ind + 1) * pt_det)

                # bbox_array_msg = get_bbox_arry_msg(tracker_arr, labels_arr, cur_img['header'])
                # pub_bbox.publish(bbox_array_msg)

                #show_frame = drawer.draw(orig_im, tracker_arr, labels_arr, (1. / (moving_tra + 1e-8)), is_tracker=True)
                det_frame = drawer.draw(orig_im, dets_arr, labels_arr, (1. /(moving_det + 1e-8)), is_tracker=False)
   
                
                testtest = None
                #adding test(to lidar)
                car_test_point = []
                t_sign_frame = cv2.cvtColor(original_img_copy, cv2.COLOR_BGR2HSV)
                sign_color = 0

                for one_arr in tracker_arr:
                    #for object
                    for num, i in enumerate(one_arr):
                        if(num == 0 or num == 1 or num == 2 or num == 3 or num == 6):
                            car_test_point.append(i)
                    #for traffic light
                
                # print(car_test_point)
                testtest = Float32MultiArray(data=car_test_point)
                if testtest is not None:
                    # print(testtest)
                    pub2.publish(testtest)

                if pub_od.get_num_connections() > 0:
                    msg = None
                    try:
                        msg = bridge.cv2_to_imgmsg(det_frame, "bgr8")
                        msg.header = cur_img['header']
                    except CvBridgeError as e:
                        print(e)
                    pub_od.publish(msg)

                frame_ind += 1
                get_new_img_msg = False
            print("####",time.time()-test_time)

    except rospy.ROSInterruptException:
        rospy.logfatal("{object_detection} is dead.")

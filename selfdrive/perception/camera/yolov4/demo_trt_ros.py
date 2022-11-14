#!/usr/bin/python3
import sys
import os
import time
import argparse
import numpy as np
import cv2
import copy
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

from tool.utils import *

from API.tracker import Tracker
from API.drawer import Drawer
from calibration import Calibration

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

class TensorRT2ROS:
    def __init__(self, abs_path, engine_path, input_size, img_shape, class_names, namesfile, interval):
        self.engine_path = engine_path
        self.input_size = input_size
        self.img_shape = img_shape
        self.class_names = class_names

        self.cur_img = {'img':None, 'header':None}
        # self.get_new_img_msg = False

        self.tracker = Tracker(img_shape, min_hits=1, num_classes=len(class_names), interval=interval)  # Initialize tracker
        self.drawer = Drawer(namesfile)
        self.calibration = Calibration(os.path.join(abs_path,'calibration_data/camera_front60.txt'), os.path.join(abs_path, 'calibration_data/old_camera_lidar.txt'))
        self.bridge = CvBridge()

        rospy.init_node('object_detection')
        rospy.Subscriber('/gmsl_camera/dev/video2/compressed', CompressedImage, self.IMG_callback)
        
        self.pub_od = rospy.Publisher('/Camera/Front60/od_result', Image, queue_size=1)
        self.pub_bbox = rospy.Publisher("/Camera/Front60/od_bbox", Float32MultiArray, queue_size=1)

    def IMG_callback(self, msg):
        s1 = time.time()
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.cur_img['img'] = img #calibration.undistort(img)
        self.cur_img['header'] = msg.header
        # self.get_new_img_msg = True

    def get_bbox_arry_msg(self, tracker_arr):
        pub_box = []
        for bbox in tracker_arr:
            pub_box.extend([bbox[0], bbox[1], bbox[2], bbox[3], bbox[6]])
        return Float32MultiArray(data=pub_box)

    def main(self):
        with get_engine(self.engine_path) as engine, engine.create_execution_context() as context:
            buffers = allocate_buffers(engine, 1) ## engine, batch size
        try:
            rate = rospy.Rate(30)
            moving_tra, moving_det = 0., 0.
            frame_ind = 0
            while not rospy.is_shutdown():
                # if self.get_new_img_msg:
                # sss = time.time()
                dets_arr, labels_arr, is_dect = None, None, None
                start = time.time()
                if np.mod(frame_ind, interval) == 0:
                    img = cv2.resize(self.cur_img['img'], self.input_size)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    orig_im = cv2.resize(copy.copy(img), (self.img_shape[1], self.img_shape[0]))
                    bbox = detect(context, buffers, img, self.input_size, self.img_shape, len(self.class_names))
                    
                    if len(bbox[0]) > 0: 
                        bbox = np.vstack(bbox)
                        bbox[:,0] = bbox[:,0] * self.img_shape[1]
                        bbox[:,1] = bbox[:,1] * self.img_shape[0]
                        bbox[:,2] = bbox[:,2] * self.img_shape[1]
                        bbox[:,3] = bbox[:,3] * self.img_shape[0]               
                        dets_arr, labels_arr = bbox[:,0:4], bbox[:,-1].astype(int)
                        
                    else:
                        dets_arr, labels_arr = np.array([]), np.array([])
                        
                    is_dect = True

                elif np.mod(frame_ind, interval) != 0:
                    dets_arr, labels_arr = np.array([]), np.array([])
                    is_dect = False

                # pt_det = (time.time() - start)
                
                tracker_arr = self.tracker.update(dets_arr, labels_arr, is_dect=is_dect)
                pt_tra = (time.time() - start)

                if frame_ind != 0:
                    moving_tra = (frame_ind / float(frame_ind + 1) * moving_tra) + (1. / float(frame_ind + 1) * pt_tra)
                    #moving_det = (frame_ind / float(frame_ind + 1) * moving_det) + (1. / float(frame_ind + 1) * pt_det)

                self.pub_bbox.publish(self.get_bbox_arry_msg(tracker_arr))

                show_frame = self.drawer.draw(orig_im, tracker_arr, labels_arr, (1. / (moving_tra + 1e-8)), is_tracker=True)
                # det_frame = self.drawer.draw(orig_im, dets_arr, labels_arr, (1. /(moving_det + 1e-8)), is_tracker=False)

                if self.pub_od.get_num_connections() > 0:
                    msg = None
                    try:
                        msg = self.bridge.cv2_to_imgmsg(show_frame, "rgb8")
                        msg.header = self.cur_img['header']
                    except CvBridgeError as e:
                        print(e)
                    self.pub_od.publish(msg)
                # self.get_new_img_msg = False
                # print("deteciont %0.4f"%(time.time()-sss))
                rate.sleep()
                frame_ind += 1

        except rospy.ROSInterruptException:
            rospy.logfatal("{object_detection} is dead.")

# Simple helper data class that's a little nicer to use than a 2-tuple.
class HostDeviceMem(object):
    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device)

    def __repr__(self):
        return self.__str__()

# Allocates all buffers required for an engine, i.e. host/device inputs/outputs.
def allocate_buffers(engine, batch_size):
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()
    for binding in engine:
        size = trt.volume(engine.get_binding_shape(binding)) * batch_size
        dims = engine.get_binding_shape(binding)
        
        # in case batch dimension is -1 (dynamic)
        if dims[0] < 0:
            size *= -1
        
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        # Allocate host and device buffers
        host_mem = cuda.pagelocked_empty(size, dtype)
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        # Append the device buffer to device bindings.
        bindings.append(int(device_mem))
        # Append to the appropriate list.
        if engine.binding_is_input(binding):
            inputs.append(HostDeviceMem(host_mem, device_mem))
        else:
            outputs.append(HostDeviceMem(host_mem, device_mem))
    return inputs, outputs, bindings, stream

# This function is generalized for multiple inputs/outputs.
# inputs and outputs are expected to be lists of HostDeviceMem objects.
def do_inference(context, bindings, inputs, outputs, stream):
    # Transfer input data to the GPU.
    [cuda.memcpy_htod_async(inp.device, inp.host, stream) for inp in inputs]
    # Run inference.
    context.execute_async(bindings=bindings, stream_handle=stream.handle)
    # Transfer predictions back from the GPU.
    [cuda.memcpy_dtoh_async(out.host, out.device, stream) for out in outputs]
    # Synchronize the stream
    stream.synchronize()
    # Return only the host outputs.
    return [out.host for out in outputs]

TRT_LOGGER = trt.Logger()

def get_engine(engine_path):
    # If a serialized engine exists, use it instead of building an engine.
    print("Reading engine from file {}".format(engine_path))
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def detect(context, buffers, image_src, input_size, img_shape, num_classes):
    IN_IMAGE_H, IN_IMAGE_W = input_size

    # Input
    resized = cv2.resize(image_src, (IN_IMAGE_W, IN_IMAGE_H), interpolation=cv2.INTER_LINEAR)
    img_in = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    img_in = np.transpose(img_in, (2, 0, 1)).astype(np.float32)
    img_in = np.expand_dims(img_in, axis=0)
    img_in /= 255.0
    img_in = np.ascontiguousarray(img_in)

    inputs, outputs, bindings, stream = buffers
    inputs[0].host = img_in

    trt_outputs = do_inference(context, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)

    trt_outputs[0] = trt_outputs[0].reshape(1, -1, 1, 4)
    trt_outputs[1] = trt_outputs[1].reshape(1, -1, num_classes)

    boxes = post_processing(img_in, 0.4, 0.6, trt_outputs, img_shape)

    return boxes
    
if __name__ == '__main__':
    abs_path = os.path.abspath(__file__)
    abs_path = abs_path.split(os.path.basename(abs_path))[0]
    
    engine_path = os.path.join(abs_path, "Yolov4_80_320.trt")
    namesfile = os.path.join(abs_path, "data/coco.names")
    input_size = (320, 320)
    class_names = load_class_names(namesfile)
    interval = 1
    img_shape = (720,1280)
    run_tensorrt = TensorRT2ROS(abs_path, engine_path, input_size, img_shape, class_names, namesfile, interval)
    run_tensorrt.main()

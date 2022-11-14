import sys
import os
import time
import argparse
import numpy as np
import cv2
import copy
# from PIL import Image
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

from tool.utils import *
from API.tracker import Tracker
from API.drawer import Drawer

try:
    # Sometimes python2 does not understand FileNotFoundError
    FileNotFoundError
except NameError:
    FileNotFoundError = IOError

def GiB(val):
    return val * 1 << 30

def find_sample_data(description="Runs a TensorRT Python sample", subfolder="", find_files=[]):
    '''
    Parses sample arguments.
    Args:
        description (str): Description of the sample.
        subfolder (str): The subfolder containing data relevant to this sample
        find_files (str): A list of filenames to find. Each filename will be replaced with an absolute path.
    Returns:
        str: Path of data directory.
    Raises:
        FileNotFoundError
    '''

    # Standard command-line arguments for all samples.
    kDEFAULT_DATA_ROOT = os.path.join(os.sep, "usr", "src", "tensorrt", "data")
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-d", "--datadir", help="Location of the TensorRT sample data directory.", default=kDEFAULT_DATA_ROOT)
    args, unknown_args = parser.parse_known_args()

    # If data directory is not specified, use the default.
    data_root = args.datadir
    # If the subfolder exists, append it to the path, otherwise use the provided path as-is.
    subfolder_path = os.path.join(data_root, subfolder)
    data_path = subfolder_path
    if not os.path.exists(subfolder_path):
        print("WARNING: " + subfolder_path + " does not exist. Trying " + data_root + " instead.")
        data_path = data_root

    # Make sure data directory exists.
    if not (os.path.exists(data_path)):
        raise FileNotFoundError(data_path + " does not exist. Please provide the correct data path with the -d option.")

    # Find all requested files.
    for index, f in enumerate(find_files):
        find_files[index] = os.path.abspath(os.path.join(data_path, f))
        if not os.path.exists(find_files[index]):
            raise FileNotFoundError(find_files[index] + " does not exist. Please provide the correct data path with the -d option.")

    return data_path, find_files

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

def image(engine_path, image_path, input_size, class_names):
    with get_engine(engine_path) as engine, engine.create_execution_context() as context:
        buffers = allocate_buffers(engine, 1)

        start = time.time()
        img = cv2.imread(image_path)
        orig_img=copy.copy(img)
        img = cv2.resize(img, (input_size[0], input_size[1]))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        bbox = detect(context, buffers, img, input_size, 80)
       
        pt_det = (time.time() - start)
        out_img = plot_boxes_cv2(orig_img, bbox, 1. / pt_det, class_names=class_names)

        cv2.imshow("detected trt_result", out_img)
        cv2.imwrite('trt_result.jpg',out_img)

        if cv2.waitKey(0) & 0xff == ord('q'):
            exit()

def video(engine_path, video_path, input_size, img_shape, class_names, tracker, drawer):
    cap = cv2.VideoCapture(video_path)
    with get_engine(engine_path) as engine, engine.create_execution_context() as context:
        buffers = allocate_buffers(engine, 1)
        moving_tra, moving_det = 0, 0
        frame_ind = 0    
        while cap.isOpened():
            re, image = cap.read()  
            img = cv2.resize(image, (input_size[0],input_size[1]))

            dets_arr, labels_arr, is_dect = None, None, None
            if np.mod(frame_ind, interval) == 0:
                orig_im = cv2.resize(copy.copy(img), (640,403))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                start = time.time()
                bbox = detect(context, buffers, img, input_size, img_shape, len(class_names))
    
                if len(bbox[0][:]) > 0: 
                    bbox = np.vstack(bbox)
                    output = copy.copy(bbox)
                    ## Use by changing width(640), height(403) to fit your test image size
                    output[:,0] = bbox[:,0] * img_shape[1]
                    output[:,1] = bbox[:,1] * img_shape[0]
                    output[:,2] = bbox[:,2] * img_shape[1]
                    output[:,3] = bbox[:,3] * img_shape[0]

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
            
            moving_tra = (frame_ind / float(frame_ind + 1) * moving_tra) + (1. / float(frame_ind + 1) * pt_tra)
            moving_det = (frame_ind / float(frame_ind + 1) * moving_det) + (1. / float(frame_ind + 1) * pt_det)

            show_frame = drawer.draw(orig_im, tracker_arr, labels_arr, (1. / (moving_tra + 1e-8)), is_tracker=True)
            #det_frame = drawer.draw(orig_im, dets_arr, labels_arr, (1. / (moving_det + 1e-8)), is_tracker=False)

            cv2.imshow('tracing result', show_frame)
        
            #cv2.imshow('detection result', det_frame)

            frame_ind += 1
            if cv2.waitKey(1) & 0xff == ord('q'):
                break

        cap.release()

    
def get_engine(engine_path):
    # If a serialized engine exists, use it instead of building an engine.
    print("Reading engine from file {}".format(engine_path))
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def detect(context, buffers, image_src, input_size, img_shape, num_classes):
    IN_IMAGE_H, IN_IMAGE_W = input_size

    ta = time.time()
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
    engine_path = "./Yolov4_80_320.trt"
    video_path = "./test.avi"
    image_path = "./data/dog.jpg"
    namesfile = 'data/coco.names'
    input_size = (320, 320)
    class_names = load_class_names(namesfile)
    interval = 1
    img_shape = (403, 640)

    tracker = Tracker(img_shape, min_hits=1, num_classes=len(class_names), interval=interval)  # Initialize tracker
    drawer = Drawer(namesfile)

    ## demo_video
    video(engine_path, video_path, input_size, img_shape, class_names, tracker, drawer)

    ## test_image  
    # image(engine_path, image_path, input_size, class_names)
    

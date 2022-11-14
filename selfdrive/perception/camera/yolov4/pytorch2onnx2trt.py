import sys
import onnx
import os
import argparse
import cv2
import torch

from models.Yolov4_model import Yolov4
from tool.utils import *
from torch.autograd import Variable
 
import onnx_tensorrt.backend as backend
import numpy as np

parser = argparse.ArgumentParser(description="Paramerters")

parser.add_argument('--weight_path', default="weight/Yolov4_epoch71_109837.pth", help="Using pre_trained weight")
parser.add_argument('--n_classes', default=7, help="Number of classes")
parser.add_argument('--gpu_num', default=0, help="Use number gpu")
parser.add_argument('--batch_size', default=1, help="Input batch size")
parser.add_argument('--cvtTensorRT', default=False, help="Direct Convert TensorRT")

args = parser.parse_args()

input_height, input_width = 320, 320

device = torch.device('cuda:%s'%(args.gpu_num) if torch.cuda.is_available() else 'cpu')
print ('Current cuda device : %s'%(torch.cuda.current_device()))



model = Yolov4(args.weight_path, n_classes=args.n_classes, inference=True, device=device)
model.to(device).eval()

input_tensor = torch.randn((args.batch_size, 3, input_height, input_width), requires_grad=True).to(device)

file_name = args.weight_path.split('.')[0].split('/')[-1]
onnx_file_name = "%s_%s_%s.onnx"%(file_name, input_height, input_width)

torch.onnx.export(model,               
                  input_tensor,                        
                  onnx_file_name,     
                  export_params=True,                     
                  opset_version=12,
                  do_constant_folding=True,
                  dynamic_axes=None,       
                  input_names = ['input'],   
                  output_names = ['boxes', 'confs'])

print('Conversion to onnx completed!')

### Convert TRT 
## ref. https://github.com/onnx/onnx-tensorrt
if args.cvtTensorRT:
    onnx_path = onnx_file_name
    model = onnx.load(onnx_path)
    engine = backend.prepare(model, device=str(device).upper())
    input_data = np.random.random(size=(1, 3, input_height, input_width)).astype(np.float32)
    output_data = engine.run(input_data)[0]

    print("output_data_shape : ",output_data.shape)

    with open(onnx_path.split('/')[-1].split('.onnx')[0]+'2_.trt', "wb") as f:
        f.write(engine.engine.engine.serialize())
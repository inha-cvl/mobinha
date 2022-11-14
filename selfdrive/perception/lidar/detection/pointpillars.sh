#!/bin/bash

export CUDA_VISIBLE_DEVICES=0

# set -x
NGPUS=1
CFG_DIR=cfgs

# CFG_NAME=voxel_rcnn/voxel_rcnn_car
CFG_NAME=kitti_models/pointpillar_wide

# python3 demo_ros.py --cfg_file cfgs/$CFG_NAME.yaml --data_path /media/sun/JS_SSD/pillar_train/pillar_add_os128_no_flip/training/velodyne --ckpt ../output/voxel_rcnn/voxel_rcnn_car/default/ckpt_os128/checkpoint_epoch_80.pth
python3.8 /home/inha/voxel_rcnn/Voxel-R-CNN/tools/demo_ros.py --cfg_file /home/inha/voxel_rcnn/Voxel-R-CNN/tools/cfgs/$CFG_NAME.yaml --data_path /home/inha/Desktop --ckpt /home/inha/voxel_rcnn/Voxel-R-CNN/output/kitti_models/pointpillar_wide/default/ckpt_pointpillar_wide_91/checkpoint_epoch_80.pth

import cv2
import numpy as np
import os
import math


class Drawer(object):
    def __init__(self, classes_path):
        self.classes_path = classes_path

        self.redColor = (0, 51, 255)
        self.whiteColor = (255, 255, 255)
        self.thickness = 1
        self.fontSize = 0.7
        self.fontFace = cv2.FONT_HERSHEY_TRIPLEX
        self.max_id = 1000

        self._init_params()

    def _init_params(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        self.labels = [c.strip() for c in class_names]
        # self.labels = ["Back", "Car", "Ped", "Cycl", "Motor", "Truck", "Bus", "Van"]

    @staticmethod
    def cal_iou(bb_test, bb_gt):
        xx1 = np.maximum(bb_test[0], bb_gt[0])
        yy1 = np.maximum(bb_test[1], bb_gt[1])
        xx2 = np.minimum(bb_test[2], bb_gt[2])
        yy2 = np.minimum(bb_test[3], bb_gt[3])
        w = np.maximum(0., xx2 - xx1)
        h = np.maximum(0., yy2 - yy1)
        wh = w * h
        iou = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1]) + (
                bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh)
        return iou

    def draw(self, show_img, dets, labels, fps, is_tracker=True):
        
        # dets for detector: x1, y1, x2, y2
        # dets for tracker: x1, y1, x2, y2, objectID, is_update, labelID
        h, w = show_img.shape[0:2]
        #show_img = img.copy()
    
        dets[dets < 0.] = 0.  # some kalman predictions results are negative
        dets = dets.astype(np.uint32)

        for idx in range(dets.shape[0]):
            cv2.rectangle(show_img, (dets[idx, 0], dets[idx, 1]), (dets[idx, 2], dets[idx, 3]),
                          self.redColor, self.thickness)

            ## car bonnet
            #cv2.rectangle(show_img, (int(img.shape[0]*0.07), int(img.shape[1]*0.8)), (int(img.shape[0]*0.93), int(img.shape[1]*0.98)),(255,255,0), 1)
            
            ## Add label
            #center_point = dets[idx,0]+(dets[idx,2]-dets[idx,0])/2, dets[idx,1]+(dets[idx,3]-dets[idx,1])/2
           
            # center_point = (math.sqrt((dets[idx,2]*dets[idx,2])-(dets[idx,0]*dets[idx,0]))/2,
            #                 math.sqrt((dets[idx,3]*dets[idx,3])-(dets[idx,1]*dets[idx,1]))/2)
            #cv2.line(show_img, (int(center_point[0]),int(center_point[1])),(show_img.shape[1]/2, int(show_img.shape[0]*0.79)),(0,255,0),1)

            if is_tracker:
                label_id = dets[idx, 6]
            else:
                label_id = labels[idx]
            label_str = self.labels[label_id]

            width, height = dets[idx, 2] - dets[idx, 0], dets[idx, 3] - dets[idx, 1]
        
            if height < 50:
                offset = 6
                label_size = cv2.getTextSize(label_str, self.fontFace, self.fontSize*0.5, self.thickness)
                bottom_left = (dets[idx, 0], dets[idx, 1] + int(0.5 * label_size[0][1]) - offset)
                cv2.rectangle(show_img, (dets[idx, 0], dets[idx, 1] - int(0.5 * label_size[0][1]) - offset),
                              (dets[idx, 0] + label_size[0][0], dets[idx, 1] + int(0.7 * label_size[0][1]) - offset),
                              self.redColor, thickness=-1)
                cv2.putText(show_img, label_str, bottom_left, self.fontFace, self.fontSize*0.5, self.whiteColor,
                            thickness=1)
                
            else:
                label_size = cv2.getTextSize(label_str, self.fontFace, self.fontSize, self.thickness)
                bottom_left = (dets[idx, 0], dets[idx, 1] + int(0.5 * label_size[0][1]))
                cv2.rectangle(show_img, (dets[idx, 0], dets[idx, 1] - int(0.5 * label_size[0][1])),
                              (dets[idx, 0] + label_size[0][0], dets[idx, 1] + int(0.7 * label_size[0][1])),
                              self.redColor, thickness=-1)
                cv2.putText(show_img, label_str, bottom_left, self.fontFace, self.fontSize, self.whiteColor,
                            thickness=1)
            
            # Add object ID
            if is_tracker:
                id_int = np.mod(dets[idx, 4], self.max_id)                

                if height < 50:
                    offset = 7
                    id_size = cv2.getTextSize(str(id_int), self.fontFace, self.fontSize*0.5, self.thickness)
                    bottom_left = (dets[idx, 2] - id_size[0][0], dets[idx, 3] + offset)
                    cv2.rectangle(show_img, (dets[idx, 2] - id_size[0][0], dets[idx, 3] - id_size[0][1] + offset),
                                  (dets[idx, 2], dets[idx, 3] + offset), self.redColor, thickness=-1)
                    cv2.putText(show_img, str(id_int), bottom_left, self.fontFace, self.fontSize*0.5, self.whiteColor, thickness=1)
                else:
                    id_size = cv2.getTextSize(str(id_int), self.fontFace, self.fontSize, self.thickness)
                    bottom_left = (dets[idx, 2] - id_size[0][0], dets[idx, 3])
                    cv2.rectangle(show_img, (dets[idx, 2] - id_size[0][0], dets[idx, 3] - id_size[0][1]),
                                  (dets[idx, 2], dets[idx, 3]), self.redColor, thickness=-1)
                    cv2.putText(show_img, str(id_int), bottom_left, self.fontFace, self.fontSize, self.whiteColor, thickness=1)

        # Draw processing time
        FPS_str = "FPS: {:.1f}".format(fps)
        inform_size = cv2.getTextSize(FPS_str, self.fontFace, self.fontSize, self.thickness)
        margin_h, margin_w = int(0.01 * h), int(0.01 * w)
        bottom_left = (margin_w, inform_size[0][1] + margin_h)
        cv2.putText(show_img, FPS_str, bottom_left, self.fontFace, self.fontSize, self.redColor, self.thickness)

        return show_img

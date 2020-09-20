#-*- encoding:utf-8 -*-
import rospkg
import os
import sys
import glob, json, time
import argparse
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
import numpy as np
import cv2
if torch.cuda.is_available():
    torch.set_default_tensor_type('torch.cuda.FloatTensor')
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("WRS_Dataset") + '/ssd.pytorch')

from ssd import build_ssd
from data import WRS2020_Detection, WRS_ROOT, AnnotationTransform
from data import WRS2020_CLASSES as labels

# ID Mapping.
# Chukyo label -> O2AC label
o2ac_label = {
    0:(1,"01-BASE"),
    1:(3,"03-PLATE2"),
    2:(2,"02-PLATE"),
    3:(4,"04_37D-GEARMOTOR-50-70"),
    4:(11,"11_MBRAC60-2-10"),
    5:(7,"07_SBARB6200ZZ_30"),
    6:(13,"13_MBGA30-2"),
    7:(13,"13_MBGA30-2"),
    8:(5,"05_MBRFA30-2-P6"),
    9:(14,"14_BGPSL6-9-L30-F7"),
    10:(8,"08_SSFHRT10-75-M4-FC55-G20"),
    11:(6,"06_MBT4-400"),
    12:(9,"09_EDCS10"),
    13:(9,"09_EDCS10"),
    14:(12,"12_CLBUS6-9-9.5"),
    15:(10,"10_CLBPS10_17_4")
}

annotation_root = rospack.get_path("WRS_Dataset") + "/Annotations/Far/Image-wise/*.json"
# アノテーションファイルの取得
annotations = glob.glob(annotation_root)

class ssd_detection():

    def __init__(self):
        fname_weight = rospack.get_path("WRS_Dataset") + "/ssd.pytorch/WRS_lr1e3_bs16.pth"
        self.net = build_ssd('test', 300, 17)    # initialize SSD
        self.net.load_weights( fname_weight )

    def object_detection(self, im_in, im_vis=None, threshold = 0.6):
        """
            Object detection by SSD
            Input:
                im_in... input_image
                threshold... threshold of detection
            Return:
                results... a list of dict(bbox, class_id, score)
        """

        # Preproc
        x = cv2.resize( im_in, (300, 300)).astype(np.float32 )
        x -= (104.0, 117.0, 123.0)
        x = x.astype(np.float32)
        x = x[:, :, ::-1].copy()
        x = torch.from_numpy(x).permute(2, 0, 1)

        #SSD forward
        xx = Variable(x.unsqueeze(0))     # wrap tensor in Variable
        if torch.cuda.is_available():
            xx = xx.cuda()
        y = self.net(xx)

        detections = y.data
        # scale each detection back up to the image
        scale = torch.Tensor(im_in.shape[1::-1]).repeat(2)
        results = list()
        for i in range(detections.size(1)):
            j = 0
            while detections[0,i,j,0] >= threshold:
                score = detections[0,i,j,0]
                label_name = labels[i-1]
                pt = (detections[0,i,j,1:]*scale).cpu().numpy()
                coords = (pt[0], pt[1]), pt[2]-pt[0]+1, pt[3]-pt[1]+1
                j+=1

                bbox = [ int(coords[0][0]), int(coords[0][1]), int(coords[1]), int(coords[2])]
                result = {"bbox": bbox, "class": i-1, "confidence": score}
                results.append( result )

        if im_vis is None:
            return results

        for res in results:
            bbox = res["bbox"]
            im_vis = cv2.rectangle( im_vis, (bbox[0],  bbox[1]),
                                    (bbox[0]+bbox[2], bbox[1]+bbox[3]),
                                    (0,255,0), 3 )
            cv2.putText( im_vis, o2ac_label[res["class"]][1],
                         (bbox[0], bbox[1]),1, 0.7, (255,255,255), 2, cv2.LINE_AA )
            cv2.putText( im_vis, o2ac_label[res["class"]][1],
                         (bbox[0], bbox[1]),1, 0.7, (255,0,0), 1, cv2.LINE_AA )

        for j in range(len(results)):
            results[j]["class"] = o2ac_label[results[j]["class"]][0]

        # cv2.imwrite("ssd_result.png", im_vis)

        return results, im_vis

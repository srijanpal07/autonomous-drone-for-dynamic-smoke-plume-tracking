#!/usr/bin/python3.8

import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from std_msgs.msg import Float64, String
import numpy as np
import cv2
import os, re, sys, datetime
from pathlib import Path
import time
import torch
from ultralytics import YOLO


print(f"Torch setup complete. Using torch {torch.__version__}",
      f"({torch.cuda.get_device_properties(0).name if torch.cuda.is_available() else 'CPU'})")


#--------------------------- OPTIONS ---------------------------#
VIEW_IMG = True
VIEW_MASK = True
SAVE_IMG = False
SAVE_FORMAT = False #'.avi', '.raw', 'jpg' or 'png'
#--------------------------- OPTIONS ---------------------------#

drone = 'drone2'



# global publisher and boundingbox
global pub, box

# global initialized variables for segmentation model
global model


#--------------------------- SET HYPER PARAMETERS ---------------------------#
MAX_DELAY = 0.5       # [seconds] delay b/w last detection and current image,
                      # after which drop images to catch up
CONF_THRES = 0.25     # previously 0.25  # confidence threshold
IOU_THRES = 0.6       # previously 0.7 # NMS IOU threshold
MAX_DET = 5           # maximum detections per image
IMGSZ = (480, 640)    # (192,224) # previously [352,448] # scaled image size to run inference on
DEVICE = 'cuda:0'     # or device = 'cpu' or 'cuda:0'
RETINA_MASK = False   # False or True (for more precise predictions)
#--------------------------- SET HYPER PARAMETERS ---------------------------#



# YOLO paths and importing
FILE = Path(__file__).resolve()
YOLOv8_ROOT = FILE.parents[1] / 'scripts/modules/yolov8-seg/'
if str(YOLOv8_ROOT) not in sys.path:
    sys.path.append(str(YOLOv8_ROOT))
YOLOv8_ROOT = Path(os.path.relpath(YOLOv8_ROOT, Path.cwd()))



def imagecallback(img):
    """
    image callback function
    """
    global pub, box, model

    # Converting image to numpy array
    img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)
    # print(f" img shape: ({img.height}, {img.width}) # (480, 640)

    if rospy.Time.now() - img.header.stamp > rospy.Duration(MAX_DELAY):
        print("Segmentation Node: dropping old image from segmentation\n")
        return
    else:
        results = model.predict(img_numpy, imgsz=IMGSZ, verbose=False, 
                                show_conf=True, device=DEVICE)

        if results[0].masks is not None:
            x_mean, y_mean = -1, -1
            max_pixel_count = 0

            for i, instantnce in enumerate(results[0].masks.data):
                mask = instantnce.cpu().numpy()
                # print(f"Mask Len: {mask.shape}") # (480, 640)

                # Pixel belonging to a segmented class is having a value of 1, rest 0
                smoke_indices = np.argwhere(mask == 1)
                smoke_pixel_count = len(smoke_indices)

                # Selecting the instance which has the maximum number of segemented pixels
                if max_pixel_count < smoke_pixel_count:
                    max_pixel_count = smoke_pixel_count
                    best_smoke_instance = i
                    indices_mean = np.mean(smoke_indices, axis=0)
                    # print(f"\n0: {indices_mean[0]} | 1: {indices_mean[1]}")
                    # 0 : inverted y axis | 1: x axis

                    x_mean, y_mean = indices_mean[1] , indices_mean[0]
                    x_mean_norm = indices_mean[1] / IMGSZ[1]
                    y_mean_norm = indices_mean[0] / IMGSZ[0]

                    print(f"x_mean_norm, y_mean_norm: {(x_mean_norm - 0.5): .3f}, {(y_mean_norm - 0.5): .3f}",
                        f"| Segmented area: {smoke_pixel_count}\n")
                    # print(f"| Image shape: {mask.shape}")
                    # mask shape: (480,640) | image length along (y, x)



            if x_mean == -1 and y_mean == -1:
                # No segmnetation data received
                box.bbox.center.x = box.bbox.center.y = -1
                box.bbox.center.theta = -1
                box.bbox.size_x = box.bbox.size_y = -1
                pub.publish(box)
            else:
                # Smoke Segmentation detected
                img_mask = results[0].masks.data[best_smoke_instance].cpu().numpy()
                img_mask = (img_mask * 255).astype("uint8")
                print(img_mask.shape) # (480, 640)

                # img_mask = cv2.cvtColor(img_mask, cv2.COLOR_GRAY2BGR)
                # print(img_mask.shape) # (480, 640, 3)

                # centroid of smoke normalized to the size of the mask
                x_mean_norm = x_mean / img_mask.shape[1]
                y_mean_norm = y_mean / img_mask.shape[0]
                # print(f"Norm (x,y): ({x_mean_norm}, {y_mean_norm})")

                # resized_mask = cv2.resize(img_mask, (192, 224), interpolation=cv2.INTER_LINEAR)

                if VIEW_MASK:
                    img_mask_resized = cv2.resize(img_mask, (320, 320))
                    cv2.imshow('Smoke Mask', img_mask_resized)
                    # cv2.imshow('Smoke Mask', img_mask)
                    # print(img_mask.shape) # (480, 640, 3)
                    cv2.waitKey(1)

                img.data = img_mask.flatten().tolist()
                img.height, img.width = img_mask.shape[0], img_mask.shape[1]

                if VIEW_IMG:
                    annotated_frame = results[0].plot()
                    annotated_frame = cv2.circle(annotated_frame, (int(x_mean), int(y_mean)),
                                                 20, (255, 0, 0), -1)

                    # Cirle at the bottom extreme right corner of the image
                    # annotated_frame = cv2.circle(annotated_frame, (640, 480),
                    #             20, (0, 255, 0), -1)
                    # Cirle at the mid extreme right corner of the image
                    # annotated_frame = cv2.circle(annotated_frame, (640, 200),
                    #             20, (0, 0, 255), -1)

                    cv2.imshow('Smoke Segmentation', annotated_frame)
                    cv2.waitKey(1)

                box.header.seq = img.header.seq
                box.header.stamp = img.header.stamp
                box.header.frame_id = ''
                box.source_img = img
                box.bbox.center.x = x_mean_norm
                box.bbox.center.y = y_mean_norm
                box.bbox.center.theta = 0
                box.bbox.size_x = smoke_pixel_count
                pub.publish(box)

        else:
            print("No Smoke Segmented!!!")
            box.bbox.center.x = -1
            box.bbox.center.y = -1
            box.bbox.center.theta = -1
            box.bbox.size_x = -1
            box.bbox.size_y = -1
            pub.publish(box)



def init_segmentation_node():
    """
    Segmentation node initilization function
    """
    global pub, box, model

    pub = rospy.Publisher('/segmentation_box', Detection2D, queue_size=1)
    box = Detection2D()

    print('Initializing YOLOv8 segmentation model')
    model= YOLO(YOLOv8_ROOT / 'yolov8-best.pt')

    # initializing node
    rospy.init_node('segmentation_node', anonymous=False)
    rospy.Subscriber(f'/{drone}/front_centre_cam', Image, imagecallback)

    rospy.spin()





if __name__ == '__main__':
    try:
        init_segmentation_node()
    except rospy.ROSInterruptException:
        pass

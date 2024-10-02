#!/usr/bin/python3.8
# license removed for brevity

#from operator import truediv
from re import sub
import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Float64, String
import numpy as np
import cv2
import os, re
import sys, datetime
import argparse
from pathlib import Path
import time
import torch


#------------------------EXECUTION SETUP------------------------#

EXECUTION = rospy.get_param('EXECUTION', default='SIMULATION') # 'SIMULATION' or 'DEPLOYMENT'
print(f"EXECUTION ==> {EXECUTION}")

if EXECUTION == 'SIMULATION':
    import airsim
    from airsim_ros_pkgs.msg import GimbalAngleEulerCmd, GPSYaw

#------------------------EXECUTION SETUP------------------------#

drone = 'drone2'

#------------------------TORCH SETUP------------------------#
# import torch_tensorrt
print(f"Torch setup complete. Using torch {torch.__version__} ({torch.cuda.get_device_properties(0).name if torch.cuda.is_available() else 'CPU'})")
#------------------------TORCH SETUP------------------------#



#------------------------OPTIONS SETUP------------------------#

target_name = 'smoke' # options: smoke,car,person
engine = False # using tensorrt
half = False
max_delay = 0.5 # [seconds] delay between last detectiona and current image after which to just drop images to catch up
conf_thres = 0.4  # confidence threshold
iou_thres = 0.45  # NMS IOU threshold

VIEW_IMG = True
VIEW_DETECTION = True
SAVE_IMG = False
SAVE_DETECTION = False
save_format = '.jpg' # originally'.avi'

#------------------------OPTIONS SETUP------------------------#

gps_t = 0



#------------------------SAVING DIRECTORY SETUP------------------------#

tmp = datetime.datetime.now()
stamp = ("%02d-%02d-%02d" % 
    (tmp.year, tmp.month, tmp.day))

if EXECUTION == 'SIMULATION':
    maindir = Path('./SavedData')
elif EXECUTION == 'DEPLOYMENT':
    username = os.getlogin()
    maindir = Path(f'/home/{username}/1FeedbackControl')

runs_today = list(maindir.glob(f'{stamp}/run*'))
if runs_today:
    runs_today = [str(name) for name in runs_today]
    regex = 'run(\d{2})'
    runs_today = [int(re.search(regex, name).group(1)) for name in runs_today]
    new_run_num = max(runs_today) + 1
else:
    new_run_num = 1

savedir = maindir.joinpath('%s/run%02d/detection' % (stamp,new_run_num))
os.makedirs(savedir)

#------------------------SAVING DIRECTORY SETUP------------------------#



#------------------------YOLO SETUP------------------------#

FILE = Path(__file__).resolve()
YOLOv5_ROOT = FILE.parents[1] / 'scripts/modules/yolov5'  # YOLOv5 root directory
if str(YOLOv5_ROOT) not in sys.path:
    sys.path.append(str(YOLOv5_ROOT))  # add YOLOv5_ROOT to PATH

YOLOv5_ROOT = Path(os.path.relpath(YOLOv5_ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device
from utils.plots import Annotator, colors
from utils.augmentations import letterbox

#------------------------YOLO SETUP------------------------#



#global publisher and boundingbox
global pub, box, video, timelog
global smoketrack_status
smoketrack_status = 'Initializing'
#global initialized variables for detection model
global imgsz, model, device, names

# labeling text on image
BLACK = (265,265,265)
font = cv2.FONT_HERSHEY_SIMPLEX
font_size = 1
font_color = BLACK
font_thickness = 2



def smoketrack_status_callback(status):
    global smoketrack_status
    #print(f"Inside Callback : {status}")
    smoketrack_status = status



def time_callback(gpstime):
    global gps_t
    gps_t = float(gpstime.time_ref.to_sec())



def imagecallback(img):
    global pub,box,video,timelog
    global imgsz, model, device, names
    global smoketrack_status

    box = Detection2D()

    img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)
    img_numpy_copy = img_numpy.copy()
    img_numpy_copy.flags.writeable = True
    img_numpy = img_numpy_copy

    if VIEW_IMG:
        text_to_image = 'img -> det node'
        img_numpy_view  = img_numpy.copy()
        img_numpy_view = cv2.putText(img_numpy_view, text_to_image, (10,30), font, font_size, font_color,
                            font_thickness, cv2.LINE_AA)
        result = img_numpy_view
        scale_percent = 100 # percent of original size
        width = int(result.shape[1] * scale_percent / 100)
        height = int(result.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(result, dim, interpolation = cv2.INTER_AREA) # resize image
        cv2.imshow('Cam Image to Detection Node', resized)
        cv2.waitKey(1)  # 1 millisecond

    if rospy.Time.now() - img.header.stamp > rospy.Duration(max_delay):
        print("DetectionNode: dropping old image from detection\n")
        return
    elif smoketrack_status == 'Initializing':
        t1 = time.time()

        # Running Detection Inference
        object,img_numpy = detection(img_numpy,imgsz,model,device,names,savenum=img.header.seq)
        t1 = time.time()
        box.header.seq = img.header.seq
        box.header.stamp = img.header.stamp
        box.header.frame_id = ''
        box.source_img = img

        if len(object) != 0 and object[0].confidence > conf_thres:
            box.bbox.center.x = object[0].bounding_box[0]
            box.bbox.center.y = object[0].bounding_box[1]
            box.bbox.center.theta = 0
            box.bbox.size_x = object[0].bounding_box[2]
            box.bbox.size_y = object[0].bounding_box[3]

            # viewing/saving images
            savenum=img.header.seq
            # text_to_image = 'processed'
            # img_numpy = cv2.putText(img_numpy,text_to_image,(10,30),font, font_size, font_color,
            #                         font_thickness, cv2.LINE_AA)

            if SAVE_DETECTION:
                if save_format=='.raw':
                    fid = open(savedir.joinpath('Detection-%06.0f.raw' % savenum),'wb')
                    fid.write(img_numpy.flatten())
                    fid.close()
                elif save_format == '.avi': video.write(img_numpy)
                else: cv2.imwrite(str(savedir.joinpath('Detection-%06.0f.jpg' % savenum)),img_numpy)
            
            if VIEW_DETECTION:
                result = img_numpy
                scale_percent = 100 # percent of original size
                width = int(result.shape[1] * scale_percent / 100)
                height = int(result.shape[0] * scale_percent / 100)
                dim = (width, height)
                resized = cv2.resize(result, dim, interpolation = cv2.INTER_AREA) # resize image
                cv2.imshow('Smoke Detection Box',resized)
                cv2.waitKey(1)  # 1 millisecond
        else:
            box.bbox.center.x = -1
            box.bbox.center.y = -1
            box.bbox.center.theta = -1
            box.bbox.size_x = -1
            box.bbox.size_y = -1
        
        # Publishing Detection Box
        pub.publish(box)

        # adding to time stamp log, every frame
        timelog.write('%d,%f,%f,%f,%f,%f,%f\n' % (img.header.seq,
                                                float(img.header.stamp.to_sec()),
                                                gps_t,
                                                box.bbox.center.x,
                                                box.bbox.center.y,
                                                box.bbox.size_x,
                                                box.bbox.size_y))
        
        # viewing/saving images
        savenum=img.header.seq      
        
        
        if SAVE_IMG:
            if save_format=='.raw':
                fid = open(savedir.joinpath('Detection-%06.0f.raw' % savenum),'wb')
                fid.write(img_numpy.flatten())
                fid.close()
            elif save_format == '.avi': video.write(img_numpy)
            else: cv2.imwrite(str(savedir.joinpath('Detection-img-%06.0f.jpg' % savenum)),img_numpy)
            
        
    else:
        print("Not Running Detection!")



def init_detection_node():
    global pub,box,video,timelog
    global smoketrack_status
    pub = rospy.Publisher(f'/{drone}/bounding_box', Detection2D, queue_size=1)
    box = Detection2D()

    # Initialize detection code before subscriber because this takes some time
    global imgsz, model, device, names
    print('Initializing YOLO model')

    if target_name == 'smoke':
        if engine:
            weights=YOLOv5_ROOT / 'smoke_BW_new_1-3-352-448.engine'
        else: weights=YOLOv5_ROOT / 'smoke.pt'
    else:
        if engine:
            weights=YOLOv5_ROOT / 'yolov5s_1-3-352-448.engine'
        else: weights=YOLOv5_ROOT / 'yolov5s.pt'

    model, device, names = detect_init(weights)

    imgsz = [352,448] # scaled image size to run inference on

    # initializing video file
    if save_format=='.avi':
        codec = cv2.VideoWriter_fourcc('M','J','P','G')
        video = cv2.VideoWriter(str(savedir.joinpath('Detection'+save_format)),
            fourcc=codec,
            fps=20,
            frameSize = (640,480)) # this size is specific to GoPro

    # initializing timelog
    timelog = open(savedir.joinpath('Metadata.csv'),'w')
    timelog.write('FrameID,Timestamp_Jetson,Timestamp_GPS,Center_x,Center_y,Width,Height\n')

    # initializing node
    rospy.init_node('detection_node', anonymous=False)
    rospy.Subscriber('/smoketrack_status', String, smoketrack_status_callback)

    if EXECUTION == 'SIMULATION':
        rospy.Subscriber(f'/{drone}/front_centre_cam', Image, imagecallback)
    if EXECUTION == 'DEPLOYMENT':
        rospy.Subscriber('/camera/image', Image, imagecallback)
        rospy.Subscriber('mavros/time_reference', TimeReference, time_callback)

    rospy.spin()



def detection(img0,imgsz,model,device,names,savenum):
    max_det=100  # maximum detections per image
    classes=None  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=False  # class-agnostic NMS
    stride = model.stride

    t1 = time.time()

    img = letterbox(img0, new_shape=imgsz,stride=stride, auto=True)[0] # Padded resize
    img = img.transpose((2, 0, 1))[::-1]  # Convert HWC to CHW, BGR to RGB
    img = np.array([img])
    img = np.ascontiguousarray(img)

    seen = 0
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img = img / 255.0  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]  # expand for batch dim
    
    t1 = time.time()
    pred = model(img)
    t1 = time.time()
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

    # initializing output list
    obj = []

    # Process predictions
    t1 = time.time()
    for i, det in enumerate(pred):  # per image
        seen += 1
        gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        
        annotator = Annotator(img0, line_width=1, example=str(names))
        
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            
            # Write results
            for *xyxy, conf, cls in reversed(det):
                # extracting bounding box, confidence level, and name for each object
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                confidence = float(conf)
                object_class = names[int(cls)]
                
                if VIEW_IMG:
                    c = int(cls)  # integer class
                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                
                # adding object to list
                obj.append(DetectedObject(np.array(xywh),confidence,object_class))

        im_with_boxes = annotator.result()


    #------return object with max confidence------------#
    if max_det != 1:
        bestobject = []
        bestconf = 0
        for ob in obj:
            if ob.object_class == target_name and ob.confidence > bestconf:
                bestobject = [ob]
                bestconf = ob.confidence  
    else:
        bestobject = [ob]
        bestconf = ob.confidence

    return bestobject,im_with_boxes



# methods from yolov5/detect_fun.py
def detect_init(weights=YOLOv5_ROOT / 'yolov5s.pt'):
	# Load model
    device='cpu' # or device='cpu' or 'cuda:0'
    device = select_device(device)

    if not engine:
        model = DetectMultiBackend(weights)    # first loads to cpu
        if half: model.half()    # then converts to half
        model.to(device)    # then sends to GPU
    else:
        model = DetectMultiBackend(weights, device=device)

    stride, names, pt = model.stride, model.names, model.pt

    return model, device, names



class DetectedObject():
    def __init__(self,xywh=[],conf=0.,cls=''):
        # normalized coordinates (by original image dimensions) of
        # horizontal center, vertical center, width, height
        self.bounding_box = xywh
        self.confidence=float(conf) # 0 is no confidence, 1 is full
        self.object_class = str(cls) # name of object (e.g., "person", or "smoke")



if __name__ == '__main__':
    try:
        init_detection_node()
    except rospy.ROSInterruptException:
        pass

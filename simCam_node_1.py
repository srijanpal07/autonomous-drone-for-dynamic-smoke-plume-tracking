#!/usr/bin/python3.8

import rospy
import airsim
import numpy as np
from sensor_msgs.msg import Image
import os
from platform import uname
import cv2


HOST = '127.0.0.1' # Standard loopback interface address (localhost)
if 'linux' in uname().system.lower() and 'microsoft' in uname().release.lower(): # In WSL2
    if 'WSL_HOST_IP' in os.environ:
        HOST = os.environ['WSL_HOST_IP']
print("Using WSL2 Host IP address: ", HOST)
drone = 'drone1'


client = airsim.MultirotorClient(ip=HOST)
client.confirmConnection()
client.enableApiControl(True)


rospy.init_node("drone1_Cam")
pub = rospy.Publisher(f"/{drone}/front_centre_cam", Image, queue_size = 1)


seq = 1
img = Image()


while not rospy.is_shutdown():
    responses = client.simGetImages([airsim.ImageRequest(f"{drone}_front_center",
                                                         airsim.ImageType.Scene,
                                                         False, False)], vehicle_name=drone)

    response = responses[0]

    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)

    img.header.seq = seq
    img.header.stamp = rospy.Time.now()
    seq += 1
    img.height,img.width = response.height, response.width
    img.data = img_rgb.flatten().tolist()

    pub.publish(img)
    rospy.sleep(0.001)

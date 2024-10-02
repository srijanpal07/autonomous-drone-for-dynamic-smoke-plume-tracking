#!/usr/bin/python3.8
# license removed for brevity

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import TimeReference,NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandTOL, CommandTOLRequest, SetMode, SetModeRequest
from std_msgs.msg import Float64
import math
import cv2
import os,re
import sys
import numpy as np
import datetime, time
from pathlib import Path


# global EXECUTION
EXECUTION = rospy.get_param('EXECUTION', default='SIMULATION') # 'SIMULATION' or 'DEPLOYMENT'

if EXECUTION == 'SIMULATION':
    import airsim
    from airsim_ros_pkgs.msg import GimbalAngleEulerCmd, GPSYaw

drone = 'drone1'

#-----------------------(srijan)----------------------------#
print_stat = False
print_stat_test = True
print_flags = True
print_state = True
print_speeds = False
#----------------------- PRINT OPTIONS ----------------------------#







# ----------------------- Deployment Parameters ------------------------------- #
yaw_center = 1500
limit_speed = 2
limit_speed_v = 2
limit_yawrate = 20
YAW_SPEED_LIMIT = 1.5
limit_max_yaw = yaw_center+500 # 2000 is 45 deg right
limit_min_yaw = yaw_center-500 # 2000 is 45 deg left


# initialize
guided_mode = False
vspeed = hspeed = fspeed = yaw = 0
yawrate = 0
alt = 10
gps_x = gps_y = gps_t = 0
gps_lat = gps_long = gps_alt = gps_alt_rel = 0
savenum = 0



def move_airsim_gimbal(pitchcommand, yawcommand):
    """
    Converts gimbal's pwm commands to angles for running is simulation
    pitchcommand - Pitch PWM. 1000 is straight ahead (0 deg) and 1900 is straight down (-90 deg) 
    yawcommand - Yaw PWM. 1000 is -45 deg and 2000 is 45 deg
    """
    global gimbal, airsim_yaw, yaw

    airsim_yaw = math.degrees(yaw)

    if airsim_yaw<0:
        airsim_yaw += 360

    gimbal_pitch = (1000 - pitchcommand) / 10
    gimbal_yaw = ((yaw_center - yawcommand) * 45) / 500
    cmd = GimbalAngleEulerCmd()
    cmd.camera_name = "drone1_front_center"
    cmd.vehicle_name = "drone1"
    cmd.pitch = gimbal_pitch
    cmd.yaw = gimbal_yaw - airsim_yaw + 90

    if cmd.yaw>=360:
        cmd.yaw = cmd.yaw % 360

    gimbal.publish(cmd)




def offboard():
    """
    Used in Simulation to set 'OFFBOARD' mode 
    so that it uses mavros commands for navigation
    """

    mode = SetModeRequest()
    mode.base_mode = 0
    mode.custom_mode = "OFFBOARD"
    print("Setting up...", end ="->")
    setm = rospy.ServiceProxy(f'/{drone}/mavros/set_mode', SetMode)
    resp = setm(mode)
    print(f' OFFBOARD MODE -> {resp}')



# checked ok
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians



# checked ok
def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion and 
    calculate quaternion components w, x, y, z
    """
    # Convert Euler angles to quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Calculate quaternion components
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return x, y, z, w



# checked ok
def pose_callback(pose):
    """
    x, y, z orientation of the drone
    """
    global yaw, alt, gps_x, gps_y

    q = pose.pose.orientation
    _, _, y = euler_from_quaternion(q.x,q.y,q.z,q.w)
    yaw = y
    alt = pose.pose.position.z
    gps_x = pose.pose.position.x
    gps_y = pose.pose.position.y



def heading_callback(heading):
    """
    Returns drone's heading with respect to North
    """
    global head
    head = heading.data



# checked ok
def compass_hdg_callback(heading):
    """
    Returns drone's heading with respect to North
    """
    global head
    head = heading.data



def imagecallback(img):
    global img_numpy

    img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)
    cv2.imshow('Master Drone View', img_numpy)
    cv2.waitKey(1)  # 1 millisecond



# checked ok
def state_callback(state):
    """
    check if drone FCU is in LOITER or GUIDED mode
    """
    global guided_mode
    global print_state

    if state.mode == 'OFFBOARD':
        guided_mode = True
        if print_flags and print_state:
            print("state.mode == 'GUIDED' -> guided_mode = True")
            print_state = False
    else:
        print(f"{state.mode}:!!!!!!!!!!!!!!!! NOT OFFBOARD")
        guided_mode = False



# checked ok
def time_callback(gpstime):
    """
    returns the gps time
    """
    global gps_t
    gps_t = float(gpstime.time_ref.to_sec())



# checked ok
def gps_callback(gpsglobal):
    """
    returns gps loaction of the drone
    gps_lat, gps_long, gps_alt
    """
    global gps_lat, gps_long, gps_alt

    gps_lat = gpsglobal.latitude
    gps_long = gpsglobal.longitude
    gps_alt = gpsglobal.altitude



# checked ok
def rel_alt_callback(altrel):
    """
    returns relative gps altitude
    """
    global gps_alt_rel

    gps_alt_rel = altrel.data




def dofeedbackcontrol():
    """
    main feedback control loop
    """
    global fspeed, hspeed, vspeed
    global twistpub, twistmsg
    global gimbal
    global yaw
    global publish_rate
    global img_numpy

    # Initialize subscribers to mavros topics
    rospy.Subscriber(f'/{drone}/mavros/state', State, state_callback)
    rospy.Subscriber(f'/{drone}/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber(f'/{drone}/mavros/time_reference', TimeReference, time_callback)
    rospy.Subscriber(f'/{drone}/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.Subscriber(f'/{drone}/mavros/global_position/rel_alt', Float64, rel_alt_callback)
    rospy.Subscriber(f'/{drone}/mavros/global_position/compass_hdg', Float64, compass_hdg_callback)
    rospy.Subscriber(f'/{drone}/front_centre_cam', Image, imagecallback)

    # Initialize Publishers
    twistpub = rospy.Publisher(f'/{drone}/mavros/setpoint_velocity/cmd_vel_unstamped',
                               Twist, queue_size=1)

    if EXECUTION == 'SIMULATION':
        gimbal = rospy.Publisher('/airsim_node/gimbal_angle_euler_cmd',
                                 GimbalAngleEulerCmd, queue_size=1)

    img_numpy = None
    savenum = 0
    tmp = datetime.datetime.now()
    stamp = ("%02d-%02d-%02d" %
        (tmp.year, tmp.month, tmp.day))
    maindir = Path('./SavedData')
    runs_today = list(maindir.glob(f'{stamp}/run*'))
    if runs_today:
        runs_today = [str(name) for name in runs_today]
        regex = 'run(\d{2})'
        runs_today = [int(re.search(regex, name).group(1)) for name in runs_today]
        new_run_num = max(runs_today) + 1
    else:
        new_run_num = 1
    savedir = maindir.joinpath('%s/run%02d/topViewSmoke' % (stamp,new_run_num))
    os.makedirs(savedir)


    publish_rate = time.time()
    twistmsg = Twist()
    rate = rospy.Rate(30)

    # feedback control loop
    while not rospy.is_shutdown():

        # speeds are initiliazed and set to 0
        fspeed, vspeed, hspeed, yaw_speed = 0, 0, 0, 0

        if not guided_mode:
            print("NOT IN GUIDED MODE!")
        else:
            print("----------------------- Hovering! -----------------------", end = '\r')
            fspeed = hspeed = vspeed = yaw_speed = 0
            move_airsim_gimbal(pitchcommand=1900, yawcommand=1500)

            if img_numpy is not None:
                savenum = savenum + 1
                if savenum % 20 ==0:
                    cv2.imwrite(str(savedir.joinpath('topViewSmoke-%06.0f.jpg' % savenum)),img_numpy)

        # bound controls to ranges
        # lower bound first, upper bound second
        fspeed = min(max(fspeed,-limit_speed),limit_speed)
        hspeed = min(max(hspeed,-limit_speed),limit_speed)
        vspeed = min(max(vspeed,-limit_speed_v),limit_speed_v)
        yaw_speed = min(max(yaw_speed,-YAW_SPEED_LIMIT),YAW_SPEED_LIMIT)

        twistmsg.linear.x = fspeed
        twistmsg.linear.y = hspeed
        twistmsg.linear.z = vspeed
        twistmsg.angular.z = yawrate

        # publishing
        twistpub.publish(twistmsg)
        publish_rate = time.time()
        rate.sleep()



if __name__ == '__main__':

    print("Initializing feedback node...")
    rospy.init_node(f'feedbackcontrol_node_{drone}', anonymous=False)
    twist_pub = rospy.Publisher(f'/{drone}/mavros/setpoint_position/local', PoseStamped, queue_size=1)

    #global EXECUTION
    print(f'Executing in ==> {EXECUTION}')
    if EXECUTION == 'SIMULATION':
        # TAKEOFF
        takeoff = CommandTOLRequest()
        takeoff.min_pitch = 0
        takeoff.yaw = 0
        takeoff.latitude = 47.641468
        takeoff.longitude = -122.140165
        takeoff.altitude = -10
        print("Taking off", end ="->")
        fly = rospy.ServiceProxy(f'/{drone}/mavros/cmd/takeoff', CommandTOL)
        resp = fly(takeoff)
        print(resp)
        time.sleep(2)

        # ARM the drone
        arm = CommandBoolRequest()
        arm.value = True
        print("Arming - 1st attempt", end ="->")
        arming = rospy.ServiceProxy(f'/{drone}/mavros/cmd/arming', CommandBool)
        resp = arming(arm)
        print(resp)
        time.sleep(5)
        print("Arming - 2nd attempt", end ="->")
        resp = arming(arm)
        print(resp)
        time.sleep(5)

        # Move to set posiiton
        print("Moving...")
        go = PoseStamped()
        go.pose.position.x = 0
        go.pose.position.y = 0
        go.pose.position.z = 20
        go.pose.orientation.z = -0.8509035
        go.pose.orientation.w = 0.525322
        twist_pub.publish(go)
        time.sleep(0.2)

        print("GOING AUTONOMOUS")
        offboard()

        # Move to set posiiton
        for i in range(75): # originally 150
            go = PoseStamped()
            go.pose.position.x = -17
            go.pose.position.y = 10
            go.pose.position.z = 250 # previuosly 250 with 55 fov of AirSim Camera
            go.pose.orientation.z = 1
            twist_pub.publish(go)
            time.sleep(0.2)

    try:
        dofeedbackcontrol()
    except rospy.ROSInterruptException:
        pass

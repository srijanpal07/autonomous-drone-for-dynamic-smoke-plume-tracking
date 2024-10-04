#!/usr/bin/python3.8

import gym
from gym import spaces
import numpy as np
import cv2
import time
import math

import rospy
from sensor_msgs.msg import TimeReference, NavSatFix
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import Detection2D
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandTOL, CommandTOLRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry

import airsim
from airsim_ros_pkgs.msg import GimbalAngleEulerCmd





# Limit gimbal parameters
AIRSIM_GIMBAL_CENTER_YAW = 1500
LIMIT_MAX_GIMBAL_YAW = AIRSIM_GIMBAL_CENTER_YAW + 500 # 2000 is 45 deg right
LIMIT_MIN_GIMBAL_YAW = AIRSIM_GIMBAL_CENTER_YAW - 500 # 2000 is 45 deg left


HOR_SPEED_TRACK = 1.5
VER_SPEED_TRACK = 2
LIMIT_SPEED = 2.5
LIMIT_V_SPEED = 4
LIMIT_YAWSPEED = 0.3
FSCAN_SPEED = 0.3
MIN_HEIGHT = 0.7

PRINT_STATE = True
DRONE = 'drone2'



class DroneEnv(gym.Env):
    """ Class representing the drone environment"""

    def __init__(self, execution='SIM', training=True):
        super(DroneEnv, self).__init__()
        self.observation_space = spaces.Box(low=0, high=255, shape=(320, 320, 1), dtype=np.uint8)

        # Action Space:
        # 2:hard-left,            1:up,                      5: hard-right
        # 2:hard-left,  3: left,  0:No movement,  4: right,  5: hard-right
        # 2:hard-left,            6:down,                    5: hard-right
        self.action_space = spaces.Discrete(7)

        self.execution = execution
        if self.execution == 'SIM':
            print("Executing in Simulation")
            import airsim
            from airsim_ros_pkgs.msg import GimbalAngleEulerCmd

        self.training = training
        if self.training:
            self.episode_count = 0
        self.print_state = True
        self.print_speed = False
        self.guided_mode = False
        self.local_x = self.local_y = self.local_z = self.drone_yaw = 0
        self.gps_t = 0
        self.gps_lat = self.gps_long = self.gps_alt = self.gps_alt_rel = 0
        self.drone_gps = [0, 0, 0]
        self.gps_alt_rel = 0
        self.drone_heading = 0
        self.time_last_bbox = None
        self.horizontalerror_bbox = self.verticalerror_bbox = 0
        self.segmented_numpy_img = None
        self.time_last_segment = None
        self.size_segment = self.horizontalerror_segment = self.verticalerror_segment = 0

        self.forward_scan = False
        self.above_smoke = True
        self.moving_to_set_alt = True
        self.set_alt_reached = False
        self.first_reset = True
        self.tracking_last_segment = True
        self.time_last_segment = None
        self.home_local_alt_received = False
        self.home_local_alt = None
        self.local_alt = None
        self.armed = False

        print("Initializing DRL Controller node...")
        rospy.init_node('drl_controller_node', anonymous=False)
        self.twist_pub = rospy.Publisher(f'/{DRONE}/mavros/setpoint_position/local',
                                         PoseStamped, queue_size=1)

        print("Initializing Drone ...")

        # Initialize subscribers to mavros topics
        rospy.Subscriber(f'/{DRONE}/mavros/state', State, self.state_callback)
        rospy.Subscriber(f'/{DRONE}/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber(f'/{DRONE}/mavros/time_reference', TimeReference, self.time_callback)
        rospy.Subscriber(f'/{DRONE}/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber(f'/{DRONE}/mavros/global_position/rel_alt', Float64, self.rel_alt_callback)
        rospy.Subscriber(f'/{DRONE}/mavros/global_position/compass_hdg', Float64, self.compass_hdg_callback)
        rospy.Subscriber(f'/{DRONE}/mavros/global_position/local', Odometry, self.local_alt_callback)

        # Initialize subscribers to image, bounding box and segmentation topics
        rospy.Subscriber(f'/{DRONE}/bounding_box', Detection2D, self.boundingbox_callback)
        rospy.Subscriber('/segmentation_box', Detection2D, self.segmentation_callback)
        print("\n-------------------------- Subscribers Initialized --------------------------")

        # Initialize Publishers
        self.guided_vel_twistpub = rospy.Publisher(f'/{DRONE}/mavros/setpoint_velocity/cmd_vel_unstamped',
                                   Twist, queue_size=1)
        self.smoketrack_status_pub = rospy.Publisher(f'/{DRONE}/smoketrack_status', String, queue_size=1)
        if execution == 'SIM':
            self.airsim_gimbal = rospy.Publisher('/airsim_node/gimbal_angle_euler_cmd',
                                GimbalAngleEulerCmd, queue_size=1)
        print("\n-------------------------- Publishers Initialized -------------------------")

        # Initialize twist message to make the drone move
        self.twistmsg = Twist()

        self.publish_rate = rospy.Rate(5)

        self.initial_setup()



    def initial_setup(self):
        """
        Initial drone setup method
        """
        print("\n---------------------------- Initializing Setup ----------------------------")
        
        if self.execution == 'SIM':
            takeoff = CommandTOLRequest()
            takeoff.min_pitch = 0
            takeoff.yaw = 0
            takeoff.latitude = 47.641468
            takeoff.longitude = -122.140165
            takeoff.altitude = -10
            print("Taking off", end ="->")
            fly = rospy.ServiceProxy(f'/{DRONE}/mavros/cmd/takeoff', CommandTOL)
            resp = fly(takeoff)
            print(resp)
            time.sleep(2)

            # Arm the drone
            arm = CommandBoolRequest()
            arm.value = True
            print("Arming - 1st attempt", end ="->")
            arming = rospy.ServiceProxy(f'/{DRONE}/mavros/cmd/arming', CommandBool)
            resp = arming(arm)
            print(resp)
            time.sleep(5)
            print("Arming - 2nd attempt", end ="->")
            resp = arming(arm)
            print(resp)
            time.sleep(5)

            print("\n---------------------------- Going Autonomous -----------------------------")
            self.offboard()

        self.reset_position()



    def reset_position(self):
        """ resetting drone position to detect smoke from top """

        print("\nResetting drone position to detect smoke from top ... ")

        self.tracking_last_segment = True
        self.time_last_segment = None

        if self.execution == 'SIM':
            if self.training:
                self.episode_count += 1
            self.above_smoke = False
            self.set_alt_reached = False
            self.forward_scan = True
            self.moving_to_set_alt = False

            if self.first_reset:
                print("\n----------------------------- First Reset ---------------------------------")
                i = 0
                while i <= 5:
                    go = PoseStamped()
                    go.pose.position.x = -17
                    go.pose.position.y = 10
                    go.pose.position.z = 250
                    go.pose.orientation.z = 1
                    self.twist_pub.publish(go)
                    i += 1
                    time.sleep(0.1)

                # i = 0
                # while i <= 120:
                #     go = PoseStamped()
                #     go.pose.position.x = -17
                #     go.pose.position.y = 10
                #     go.pose.position.z = 250
                #     go.pose.orientation.z = 1
                #     self.twist_pub.publish(go)
                #     i += 1
                #     time.sleep(0.2)

                # Initlally pitch gimbal down to scan for smoke underneath
                self.move_airsim_gimbal(pitchcommand = 1900, yawcommand = 1500)
                self.first_reset = False
                print("-------------------------- First Reset Done -------------------------------\n")
            else:
                print("\n-------------------------------- Reset ------------------------------------")
                i = 0
                while i <= 120:
                    go = PoseStamped()
                    go.pose.position.x = -17
                    go.pose.position.y = 10
                    go.pose.position.z = 250
                    go.pose.orientation.z = 1
                    self.twist_pub.publish(go)
                    i += 1
                    time.sleep(0.2)

            # Initlally pitch gimbal down to scan for smoke underneath
            self.move_airsim_gimbal(pitchcommand = 1900, yawcommand = 1500)
            while not rospy.is_shutdown():
                if (self.time_last_bbox is not None and
                    ((rospy.Time.now() - self.time_last_bbox) < rospy.Duration(.5))):
                    self.forward_scan = False
                    self.above_smoke = True
                    self.moving_to_set_alt = True
                    self.set_alt_reached = False
                elif not self.above_smoke:
                    # Keep gimbal pitched down
                    self.move_airsim_gimbal(pitchcommand = 1900, yawcommand = 1500)
                    if self.forward_scan:
                        # Drone will keep moving forward until it sees the smoke below it
                        fspeed = FSCAN_SPEED
                        vspeed = hspeed = 0
                        self.move_drone(for_speed=fspeed, ver_speed=vspeed, hor_speed=hspeed)

                if self.above_smoke and self.moving_to_set_alt:
                    fspeed, hspeed, vspeed = self.move_to_set_alt()
                    self.move_drone(for_speed=fspeed, ver_speed=vspeed, hor_speed=hspeed)

                if not self.moving_to_set_alt and self.set_alt_reached:
                    # Pitch the gimbal straight ahead to start tracking smoke
                    self.move_airsim_gimbal(pitchcommand = 1000, yawcommand = 1500)
                    print("Done Ressetting to initial drone position\n")
                    break
        else:
            if self.first_reset:
                print("\n[DEPLOYMENT] ----------------------------- First Reset ---------------------------------")
                self.first_reset = False
                print("[DEPLOYMENT] -------------------------- First Reset Done -------------------------------\n")
            else:
                print("\n[DEPLOYMENT] -------------------------------- Reset ------------------------------------")
                while not rospy.is_shutdown():
                    if self.guided_mode:
                        print("[DEPLOYMENT] Done Ressetting to initial drone position\n")
                        break
                    else:
                        print("[DEPLOYMENT] Waiting for guided mode!")



    def move_airsim_gimbal(self, pitchcommand, yawcommand):
        """
        Converts gimbal's pwm commands to angles for running is simulation
        pitchcommand - Pitch PWM. 1000 is straight ahead (0 deg) and 1900 is straight down (-90 deg) 
        yawcommand - Yaw PWM. 1000 is -45 deg and 2000 is 45 deg
        """
        if self.execution == 'SIM':
            airsim_yaw = math.degrees(self.drone_yaw)
            if airsim_yaw < 0:
                airsim_yaw += 360

            gimbal_pitch = (1000 - pitchcommand) / 10
            gimbal_yaw = ((AIRSIM_GIMBAL_CENTER_YAW - yawcommand) * 45) / 500
            cmd = GimbalAngleEulerCmd()
            cmd.camera_name = f"{DRONE}_front_center"
            cmd.vehicle_name = DRONE
            cmd.pitch = gimbal_pitch
            cmd.yaw = gimbal_yaw - airsim_yaw + 90

            if cmd.yaw>=360:
                cmd.yaw = cmd.yaw % 360

            self.airsim_gimbal.publish(cmd)



    def state_callback(self, state):
        """
        check if drone FCU is in LOITER or OFFBOARD mode
        """
        if self.execution == 'SIM':
            if state.armed:
                self.armed = True
            else:
                print(f"[SIM] Armed -> {state.armed}")

            if state.mode == 'OFFBOARD':
                self.guided_mode = True
                if self.print_state:
                    print("state.mode == 'OFFBOARD' -> guided_mode = True")
                    self.print_state = False
            else:
                print(f"{state.mode}:!!!!!!!!!!!!!!!! NOT OFFBOARD")
                self.guided_mode = False
        else:
            if state.armed:
                self.armed = True
            else:
                print(f"[DEPLYMENT] Armed -> {state.armed}")

            if state.mode == 'GUIDED':
                self.guided_mode = True
                if self.print_state:
                    print("[DEPLOYMENT] state.mode == 'GUIDED' -> guided_mode = True")
                    self.print_state = False
            else:
                print(f"[DEPLOYMENT] {state.mode}:!!!!!!!!!!!!!!!! NOT GUIDED")
                self.guided_mode = False



    def offboard(self):
        """
        Used in Simulation to set 'OFFBOARD' mode 
        so that drone uses mavros commands for navigation
        """
        if self.execution == 'SIM':
            mode = SetModeRequest()
            mode.base_mode = 0
            mode.custom_mode = "OFFBOARD"
            print("Setting up...", end ="->")
            setm = rospy.ServiceProxy(f'/{DRONE}/mavros/set_mode', SetMode)
            resp = setm(mode)
            print(f' OFFBOARD MODE -> {resp}')



    def euler_from_quaternion(self, x, y, z, w):
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



    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion and 
        Calculate quaternion components w, x, y, z
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



    def pose_callback(self, pose):
        """
        x, y, z orientation of the drone
        """

        self.local_x = pose.pose.position.x
        self.local_y = pose.pose.position.y
        self.local_z = pose.pose.position.z
        q = pose.pose.orientation
        _, _, y = self.euler_from_quaternion(q.x,q.y,q.z,q.w)
        self.drone_yaw = y


    def local_alt_callback(self, gpsglobal):
        """
        local altitude callback
        """
        self.local_alt = gpsglobal.pose.pose.position.z
        # if self.execution != 'SIM':
        if not self.home_local_alt_received and self.armed:
            self.home_local_alt = self.local_alt
            self.home_local_alt_received = True
            print(f"Home height (local_z) set at arming: {self.home_local_alt: .5f}")



    def time_callback(self, gps_time):
        """
        Returns the gps time
        """
        self.gps_t = float(gps_time.time_ref.to_sec())



    def gps_callback(self, gps_global):
        """
        Returns gps loaction of the drone
        gps_lat, gps_long, gps_alt and 
        drone_gps = [gps_lat, gpa_long, gps_alt]
        """

        self.gps_lat = gps_global.latitude
        self.gps_long = gps_global.longitude
        self.gps_alt = gps_global.altitude
        self.drone_gps[0], self.drone_gps[1], self.drone_gps[2] = self.gps_lat, self.gps_long, self.gps_alt



    def rel_alt_callback(self, altrel):
        """
        Returns relative gps altitude
        """
        self.gps_alt_rel = altrel.data



    def compass_hdg_callback(self, heading):
        """
        Returns drone's heading with respect to North
        """
        self.drone_heading = heading.data



    def boundingbox_callback(self, box):
        """
        Smoke detection bounding box callback function
        """

        # positive errors give right, up
        if box.bbox.center.x != -1:
            self.time_last_bbox = rospy.Time.now()

            # if box center is on LHS of image, error is positive
            self.horizontalerror_bbox = .5-box.bbox.center.x

            # if box center is on upper half of image, error is positive
            self.verticalerror_bbox = .5-box.bbox.center.y



    def segmentation_callback(self, box):
        """
        Smoke segmentation callback function
        """

        self.size_segment = box.bbox.size_x
        #print(f"size of the segment: {self.size_segment}")

        # positive errors give right, up
        if box.bbox.center.x != -1 and box.bbox.size_x > 200:
            self.time_last_segment = rospy.Time.now()
            # if box center is on LHS of image, error is positive
            self.horizontalerror_segment = box.bbox.center.x - 0.5
            # if box center is on upper half of image, error is positive
            self.verticalerror_segment = 0.5 - box.bbox.center.y

            segmented_numpy_img_flattened = np.frombuffer(box.source_img.data, dtype=np.uint8)
            segmented_numpy_img_large = segmented_numpy_img_flattened.reshape(
                box.source_img.height, box.source_img.width,-1)

            segmented_numpy_img_ = cv2.resize(segmented_numpy_img_large, (320, 320))
            if segmented_numpy_img_.ndim == 2:
                self.segmented_numpy_img = segmented_numpy_img_[:, :, np.newaxis]
            else:
                self.segmented_numpy_img = segmented_numpy_img_

        else:
            self.horizontalerror_segment = self.verticalerror_segment = 0
            self.size_segment = 0



    def step(self, action):
        """
        Applies the action to the drone (sets horizontal and vertical velocities).
        Gets a new observation.
        Calculates the reward and checks if the episode is done.
        """
        self.move_airsim_gimbal(pitchcommand = 1000, yawcommand = 1500)
        skip, done = self.check_skip_and_done_condition()

        if skip and not done:
            # Drone out of smoke --> Skip the step
            reward = 0
            self.move_drone(for_speed=0.1, ver_speed=0, hor_speed=0)
            obs = self.get_observation()

            return obs, reward, done, {}

        elif done:
            # Drone out of smoke for more than 5secs --> Episode is done
            reward = 0
            obs = self.get_observation()

            return obs, reward, done, {}

        elif not skip and not done:
            self.smoketrack_status_pub.publish("Tracking Smoke")
            print("\n---------------------------------------------------------------")
            print(f"DRL Predicted Actions: {action}")

            reward = self.take_action(action)

            obs = self.get_observation()

            return obs, reward, done, {}



    def reset(self): # seed=None
        """
        Reset the drone to its initial state
        """
        # Resets the drone to the smoke dispersion region where smoke tracking starts
        self.reset_position()

        # Return the initial observation
        return self.get_observation()



    def get_observation(self):
        """
        Capture the frame from the drone's camera
        """

        frame = self.segmented_numpy_img
        return frame



    def move_to_set_alt(self):
        """
        Make the drone to move to setpoint altitude
        """

        if self.moving_to_set_alt:
            if self.gps_alt_rel > 15:
                fspeed = (self.verticalerror_bbox-0.10) * (5)
                hspeed = (self.horizontalerror_bbox) * (5) # +0.1
                vspeed = -3
                print("Coming down to setpoint altitude!", end="\r")
            elif self.gps_alt_rel < 6.5:
                fspeed = hspeed = vspeed = 0
                self.moving_to_set_alt = False
                self.set_alt_reached = True
                print(f"\nSet point altitude reached! drone_yaw: {self.drone_yaw}\n")
                self.move_airsim_gimbal(pitchcommand = 1000, yawcommand = 1500)
                # self.move_airsim_gimbal(pitchcommand = 1000, yawcommand = 1500)
            else:
                fspeed = hspeed = 0
                vspeed = -5

        return fspeed, hspeed, vspeed



    def move_drone(self, for_speed=0, hor_speed=0, ver_speed=0, yaw_speed=0):
        """
        Function to make the drone move
        """

        # bound controls to ranges
        # lower bound first, upper bound second
        for_speed = min(max(for_speed, -LIMIT_SPEED), LIMIT_SPEED)
        hor_speed = min(max(hor_speed, -LIMIT_SPEED), LIMIT_SPEED)
        ver_speed = min(max(ver_speed, -LIMIT_V_SPEED), LIMIT_V_SPEED)
        yaw_speed = min(max(yaw_speed, -LIMIT_YAWSPEED), LIMIT_YAWSPEED)

        self.twistmsg.linear.x = for_speed
        self.twistmsg.linear.y = hor_speed
        self.twistmsg.linear.z = ver_speed
        self.twistmsg.angular.z = yaw_speed

        if self.print_speed:
            if self.execution == 'SIM' and not self.moving_to_set_alt:
                print(f"fspeed: {for_speed: .3f} | hspeed: {hor_speed: .3f}",
                    f"| vspeed: {ver_speed: .3f}")
            else:
                print(f"fspeed: {for_speed: .3f} | hspeed: {hor_speed: .3f}",
                    f"| vspeed: {ver_speed: .3f}")


        # If the drone is too close to the ground
        if (self.local_alt and self.home_local_alt) is not None:
            if self.local_alt - self.home_local_alt < MIN_HEIGHT:
                print("\n!!! Below min_height !!!\n")
                ver_speed = 0
                self.twistmsg.linear.z = ver_speed

        # publishing
        self.guided_vel_twistpub.publish(self.twistmsg)



    def take_action(self, action):
        """ 
        Commanding drone to perform autonomus velocities from DRL controller based on prediction
        # Action Space:
        # 2:hard-left,            1:up,                      5: hard-right
        # 2:hard-left,  3: left,  0:No movement,  4: right,  5: hard-right
        # 2:hard-left,            6:down,                    5: hard-right
        """

        fspeed = 1 # maintaining constant forward and vertical speed
        if action == 0:
            vspeed, hspeed = 0, 0                      # No movement
            print("[DRL ACTION] : No movement")
        elif action == 1:
            vspeed, hspeed = VER_SPEED_TRACK, 0        # up
            print("[DRL ACTION] : Move up")
        elif action == 2:
            vspeed, hspeed = 0, HOR_SPEED_TRACK*2      # hard-left
            print("[DRL ACTION] : Move hard-left")
        elif action == 3:
            vspeed, hspeed = 0, HOR_SPEED_TRACK        # left
            print("[DRL ACTION] : Move left")
        elif action == 4:
            vspeed, hspeed = 0, -HOR_SPEED_TRACK       # right
            print("[DRL ACTION] : Move right")
        elif action == 5:
            vspeed, hspeed = 0, -HOR_SPEED_TRACK*2     # hard-right
            print("[DRL ACTION] : Move hard-right")
        elif action == 6:
            vspeed, hspeed = -VER_SPEED_TRACK, 0       # down
            print("[DRL ACTION] : Move down")


        # if PID is used insted of DRL output
        if self.training:
            hspeed_pid, vspeed_pid = (self.horizontalerror_segment*-5), (self.verticalerror_segment*7.5)
            print(f"[PID ACTION] : (vspeed, hspeed) = ({vspeed_pid: .3f}, {hspeed_pid: .3f})")
            if self.episode_count%5==0:
                print("Drone is controlled Using PID")
                hspeed, vspeed = hspeed_pid, vspeed_pid

        hspeed_pid, vspeed_pid = (self.horizontalerror_segment*-3), (self.verticalerror_segment*5)
        print(f"[PID ACTION] : (vspeed, hspeed) = ({vspeed_pid: .3f}, {hspeed_pid: .3f})")
        print("Drone is controlled Using PID")
        hspeed, vspeed = hspeed_pid, vspeed_pid

        self.move_drone(for_speed=fspeed, ver_speed=vspeed, hor_speed=hspeed)

        # calculate reward and done
        reward = self.calculate_reward(action)

        return reward



    def calculate_reward(self, action):
        """
        Calculate reward for predicted movements of the DRL controller
        """
        # Action Space:
        # 2:hard-left,            1:up,                      5: hard-right
        # 2:hard-left,  3: left,  0:No movement,  4: right,  5: hard-right
        # 2:hard-left,            6:down,                    5: hard-right
        vertical_line = 0.25
        horizontal_line = 0.15

        if abs(self.horizontalerror_segment) < vertical_line:
            if abs(self.verticalerror_segment) < horizontal_line:
                if abs(self.horizontalerror_segment) < 0.02:
                    if action == 0:
                        reward = 1
                    elif action == (3 or 4):
                        reward = 0
                    else:
                        reward = -1
                elif self.horizontalerror_segment < -0.02:
                    if action == 3:
                        reward = 1
                    elif action == 2:
                        reward = 0.5
                    else:
                        reward = -1
                elif self.horizontalerror_segment > 0.02:
                    if action == 4:
                        reward = 1
                    elif action == 5:
                        reward = 0.5
                    else:
                        reward = -1
                else:
                    print("No valid reward")
            elif self.verticalerror_segment > horizontal_line:
                if action == 1:
                    reward = 1
                # elif action == (2 or 3) and self.horizontalerror_segment < vertical_line:
                #     reward = 0.5
                # elif action == (4 or 5) and self.horizontalerror_segment > -vertical_line:
                #     reward = 0.5
                else:
                    reward = -1
            elif (self.verticalerror_segment) < -(horizontal_line):
                if action == 6:
                    reward = 1
                # elif action == (2 or 3) and self.horizontalerror_segment < vertical_line:
                #     reward = 0.5
                # elif action == (4 or 5) and self.horizontalerror_segment > -vertical_line:
                #     reward = 0.5
                else:
                    reward = -1
        elif self.horizontalerror_segment < -(vertical_line):
            if action == 2:
                reward = 1
            elif action == 3:
                reward = 0.5
            elif action == 1 and self.verticalerror_segment > horizontal_line:
                reward = 1
            elif action == 6 and self.verticalerror_segment < -horizontal_line:
                reward = 1
            else:
                reward = -1
        elif self.horizontalerror_segment > vertical_line:
            if action == 5:
                reward = 1
            elif action == 4:
                reward = 0.5
            elif action == 1 and self.verticalerror_segment > horizontal_line:
                reward = 1
            elif action == 6 and self.verticalerror_segment < -horizontal_line:
                reward = 1
            else:
                reward = -1


        print(f"[Reward] reward: {reward} | ",
              f"centroid (hor, ver): ({self.horizontalerror_segment: .5f}, ",
              f"{self.verticalerror_segment: .5f}) | DRL Predicted Action: {action}")

        return reward



    def check_skip_and_done_condition(self):
        """
        Returns True if the episode is over (the smoke is not present close)
        otherwise returns False
        """

        if self.size_segment < 100:
            skip = True
            print(f"size of segment: {self.size_segment} | skip: {skip}")
        else:
            skip = False
            print(f"size of segment: {self.size_segment} | Not Skipped")


        if self.size_segment < 100:
            if self.tracking_last_segment:
                # print("Tracking last segment time!")
                self.time_last_segment = rospy.Time.now()
                self.tracking_last_segment = False
                done = False
            else:
                if self.execution=='SIM' and ((rospy.Time.now() - self.time_last_segment)
                    > rospy.Duration(2)):
                    done = True
                    print(f"2secs past from the last detected smoke | done: {done}")
                    self.smoketrack_status_pub.publish("Smoke Tracking Done")
                elif self.execution=='DEPLOY' and ((rospy.Time.now() - self.time_last_segment)
                    > rospy.Duration(5e15)):
                    done = True
                    print(f"[DEPLOYMENT] 5e15 secs past from the last detected smoke | done: {done}")
                else:
                    done = False
        else:
            # print("last segment time lost!")
            self.tracking_last_segment = True
            done = False


        return skip, done

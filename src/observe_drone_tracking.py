import numpy as np
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import random, math, cv2
from scipy.interpolate import splprep, splev



R_EARTH = 6371e3 # in meters - must be in meters
sin = np.sin
cos = np.cos
asin = np.arcsin
acos = np.arccos
atan2 = np.arctan2
HEADING = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480



class ObserveDroneTracking:
    """Class for observing drone tracking smoke and running evaluation mode"""

    def __init__(self, fx=417.0321044921875, fy=417.0321044921875,
                 observer_drone="drone1", tracker_drone="drone2"):
        self.fx = fx
        self.fy = fy
        self.observer_drone = observer_drone
        self.tracker_drone = tracker_drone

        self.this_drone_guided_mode = False
        self.observer_drone_guided_mode = False
        self.tracking_smoke = False
        self.smoke_tracking_done = False
        self.yaw = 0
        self.count = 1
        self.check = 0
        self.threshold = 210

        self.observer_drone_data = {}
        self.tracker_drone_data = {}
        self.dlat = None
        self.dlon = None
        self.save_data = False
        self.img_numpy = None
        self.largest_contour = None
        self.smoke_contour_img = None
        self.tracker_drone_pixel_coords = None
        self.smoke_skeleton_mean_spline = None
        self.tracker_drone_pixel_coords_smooth = None
        self.tracking_start_loc = None
        self.source_loc = None

        self.x_positions = []
        self.y_positions = []

        self.drone_dist_from_mline = []
        self.drone_dist_from_closest_contour = []
        self.drone_within_smoke_contour = []

        rospy.init_node(f'observer_node_{observer_drone}', anonymous = False)

        # Initializing Observer Drone Subscribers
        rospy.Subscriber(f'/{self.observer_drone}/mavros/state', State, self.observer_drone_state_callback)
        rospy.Subscriber(f'/{self.observer_drone}/mavros/global_position/global', NavSatFix, self.observer_drone_gps_callback)
        rospy.Subscriber(f'/{self.observer_drone}/mavros/local_position/pose', PoseStamped, self.observer_drone_pose_callback)
        rospy.Subscriber(f'/{self.observer_drone}/mavros/global_position/local', Odometry, self.observer_drone_local_altitude_callback)
        rospy.Subscriber(f'/{self.observer_drone}/mavros/global_position/compass_hdg', Float64, self.observer_drone_compass_heading_callback)

        rospy.Subscriber(f'/{self.observer_drone}/front_centre_cam', Image, self.imagecallback)

        # Initilaizing Tracker Drone Subscribers
        rospy.Subscriber(f'/{self.tracker_drone}/mavros/global_position/global', NavSatFix, self.tracker_drone_gps_callback)
        rospy.Subscriber(f'/{self.tracker_drone}/mavros/local_position/pose', PoseStamped, self.tracker_drone_pose_callback)
        rospy.Subscriber(f'/{self.tracker_drone}/mavros/global_position/local', Odometry, self.tracker_drone_local_altitude_callback)
        rospy.Subscriber(f'/{self.tracker_drone}/smoketrack_status', String, self.tracker_drone_status_callback)

        print(" ------------------ Observer Node Initilized ------------------ ")



    def imagecallback(self, img):
        """
        observer drone image callback function
        """
        if True: # self.tracking_smoke:
            img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)
            self.img_numpy = np.copy(img_numpy)

            if self.img_numpy.dtype != np.uint8 and self.img_numpy.dtype != np.float32:
                self.img_numpy = self.img_numpy.astype(np.uint8)

            # Finding the smoke Skeleton in the top-view image of the obserever drone
            img_numpy_grayscale = cv2.cvtColor(self.img_numpy, cv2.COLOR_BGR2GRAY)
            _, thresholded_img = cv2.threshold(img_numpy_grayscale,
                                            self.threshold, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresholded_img, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            self.largest_contour = max(contours, key=cv2.contourArea)
            mask = np.zeros_like(thresholded_img)
            cv2.drawContours(mask, [self.largest_contour], -1, (255), thickness=cv2.FILLED)
            smoke_contour = cv2.bitwise_and(thresholded_img, mask)

            # Calculate mean line of the smoke contour (skeleton)
            x_mean, y_mean = self.calculate_mean_line(smoke_contour)

            # Fit a spline to the points
            tck, _ = splprep([x_mean, y_mean], s=0)
            unew = np.linspace(0, 1.0, 1000)
            out = splev(unew, tck)

            # Convert spline output to an array of points
            spline_points = np.vstack((out[0], out[1])).T
            self.smoke_skeleton_mean_spline = np.int32(spline_points)
            # print("\nSpline::")
            # print(len(self.smoke_skeleton_mean_spline))
            # print("Distance of closest spline point to (0,0): ")
            # print(self.closest_distance_to_spline_vectorized((0, 0), self.smoke_skeleton_mean_spline))

            # Showing the smoke contour with the mean line (spline)
            self.smoke_contour_img = cv2.cvtColor(smoke_contour, cv2.COLOR_GRAY2RGB)
            cv2.polylines(self.smoke_contour_img, [self.smoke_skeleton_mean_spline], isClosed=False,
                        color=(0, 255, 0), thickness=2)
            cv2.imshow("Smoke Skeleton (with mean line)", self.smoke_contour_img)
            cv2.waitKey(1)


            #  Showing tracker drone location in the observer drone image
            if self.tracker_drone_pixel_coords is not None:
                tracker_drone_pixel_coords = (int(self.tracker_drone_pixel_coords[0]),
                                                int(self.tracker_drone_pixel_coords[1]))

                # Smoothing the movements of the drone location in the image frame
                self.x_positions.append(tracker_drone_pixel_coords[0])
                self.y_positions.append(tracker_drone_pixel_coords[1])
                smoothed_x = self.exponential_moving_average(np.array(self.x_positions))
                smoothed_y = self.exponential_moving_average(np.array(self.y_positions))
                self.tracker_drone_pixel_coords_smooth = (int(smoothed_y[-1]), int(smoothed_x[-1]))

                

                # Showing smoke countour with mean line with tracker drone location
                smoke_contour_img_with_drone_loc = cv2.circle(self.smoke_contour_img,
                                                            self.tracker_drone_pixel_coords_smooth,
                                                            3, (0, 0, 255), 3)
                cv2.imshow('Smoke Skeleton (with tracker drone location)',
                        smoke_contour_img_with_drone_loc)
                cv2.waitKey(1)

                # Showing tracker drone location on observer drone image
                image_view = cv2.circle(self.img_numpy, self.tracker_drone_pixel_coords_smooth,
                                        3, (0, 0, 255), 3) # radius, color, thickness
                cv2.imshow('Smoke Tracking (top view with tracker drone location)', image_view)
                cv2.waitKey(1)

        else:
            print("Drone not yet tracking smoke!", end='\r')



    def collect_data_to_evaluate_tracking(self):
        """
        measure tracker drone distance drome the smean line of the smoke
        """

        # target_y = self.tracker_drone_pixel_coords_smooth[1]
        # index = np.argmin(np.abs(self.smoke_skeleton_mean_spline[:, 1] - target_y))
        # x_at_target_y = self.smoke_skeleton_mean_spline[index, 0]

        # dist_from_mline = x_at_target_y - self.tracker_drone_pixel_coords_smooth[0]
        dist_from_mline = self.closest_distance_to_spline_vectorized(self.tracker_drone_pixel_coords_smooth, self.smoke_skeleton_mean_spline)
        # print(f"Distance of the drone from slpine: {dist_from_mline: .2f}")
        self.drone_dist_from_mline.append(abs(dist_from_mline))

        result = cv2.pointPolygonTest(self.largest_contour,
                                      self.tracker_drone_pixel_coords_smooth, True)

        if result > 0:
            self.drone_within_smoke_contour.append(1)
        elif result == 0:
            self.drone_within_smoke_contour.append(1)
        else:
            self.drone_within_smoke_contour.append(0)
            self.drone_dist_from_closest_contour.append(abs(result))



    def calculate_mean_line(self, smoke_segment):
        """
        calculating the mean line of the smoke skeleton
        """
        smoke_indices = np.where(smoke_segment == 255)
        unique_y = np.unique(smoke_indices[0])
        mean_x_per_y = [np.mean(smoke_indices[1][smoke_indices[0] == y]) for y in unique_y]

        for i, value in enumerate(mean_x_per_y):
            if np.isnan(value):
                mean_x_per_y[i] = 0

        return mean_x_per_y, unique_y



    def closest_distance_to_spline_vectorized(self, pixel, spline_points):
        # Extract pixel coordinates
        pixel_x, pixel_y = pixel
        
        # Compute the squared Euclidean distances
        distances = (spline_points[:, 0] - pixel_x)**2 + (spline_points[:, 1] - pixel_y)**2
        
        # Find the minimum distance and return the square root of it (actual Euclidean distance)
        min_distance = np.sqrt(np.min(distances))
        
        return min_distance



    def exponential_moving_average(self, data, alpha=0.3):
        """
        for smoothing out the movements of the drone location in the image
        """
        ema = np.zeros_like(data)
        ema[0] = data[0]
        for i in range(1, len(data)):
            ema[i] = alpha * data[i] + (1 - alpha) * ema[i - 1]

        return ema



    def observer_drone_state_callback(self, state):
        """
        check if observer drone FCU is in loiter or guided mode
        """
        if state.mode == 'GUIDED':
            self.observer_drone_guided_mode = True
            print(state.mode, end='\r')
            self.check = self.check+1
        elif state.mode != 'GUIDED':
            self.observer_drone_guided_mode= True



    def observer_drone_gps_callback(self, gpsglobal):
        """
        latitude, longitude, altitude, and heading of observer drone
        """
        self.observer_drone_data['lat'] = gpsglobal.latitude
        self.observer_drone_data['lon'] = gpsglobal.longitude
        self.observer_drone_data['alt'] = gpsglobal.altitude
        self.observer_drone_data['heading'] =  HEADING+self.yaw



    def observer_drone_pose_callback(self, pose):
        """
        x, y, z orientation of observer drone
        """
        self.observer_drone_data['x'] = pose.pose.position.x
        self.observer_drone_data['y'] = pose.pose.position.y
        self.observer_drone_data['z']= pose.pose.position.z



    def observer_drone_local_altitude_callback(self, gpsglobal):
        """
        global_position/local altitude of observer drone
        """
        self.observer_drone_data['Altitude'] = gpsglobal.pose.pose.position.z



    def observer_drone_compass_heading_callback(self, pose):
        """
        yaw of observer drone
        """
        self.yaw = pose.data



    def tracker_drone_gps_callback(self, gpsglobal):
        """
        latitude, longitude, altitude, and heading of tracker drone
        """
        self.tracker_drone_data['lat'] = gpsglobal.latitude
        self.tracker_drone_data['lon'] = gpsglobal.longitude
        self.tracker_drone_data['alt'] = gpsglobal.altitude



    def tracker_drone_pose_callback(self, pose):
        """
        x, y, z orientation of drone tracking smoke
        """
        self.tracker_drone_data['x'] = pose.pose.position.x
        self.tracker_drone_data['y'] = pose.pose.position.y
        self.tracker_drone_data['z'] = pose.pose.position.z



    def tracker_drone_local_altitude_callback(self, gpsglobal):
        """
        global_position/local altitude of tracker drone
        """
        self.observer_drone_data['Altitude_tracker'] = gpsglobal.pose.pose.position.z



    def tracker_drone_status_callback(self, status):
        """
        tracking status callback function
        """
        if status.data == "Tracking Smoke":
            self.tracking_smoke = True
            self.smoke_tracking_done = False
        elif status.data == "Smoke Tracking Done":
            self.tracking_smoke = False
            self.smoke_tracking_done = True
        else:
            self.tracking_smoke = False
            self.smoke_tracking_done = False



    def show_tracker_drone_in_observer_image(self):
        """
        showing the location of the tracker drone
        in the top-dowm view of the observer drone image
        """
        while not rospy.is_shutdown():
            try:
                t_mat = self.mapping_2d_3d(self.observer_drone_data)
                t_mat_inv = np.linalg.inv(t_mat)

                # converting the gps coordinate of the tracker drone
                # to pixel coordinate using the transforation matrix
                gps_coord = np.array([self.tracker_drone_data['lat'],
                                      self.tracker_drone_data['lon'], 1])
                self.tracker_drone_pixel_coords = np.matmul(t_mat_inv, gps_coord)

                if self.tracking_smoke:
                    self.collect_data_to_evaluate_tracking()

                    if self.tracking_start_loc is None:
                        self.tracking_start_loc = self.tracker_drone_pixel_coords
                        print("\nSmoke tracking starting Location - ")
                        print(self.tracking_start_loc)

                if not self.tracking_smoke and self.smoke_tracking_done:

                    self.source_loc = self.tracker_drone_pixel_coords
                    print("\nSmoke Source Location - ")
                    print(self.source_loc)

                    tracking_distance = math.sqrt((self.source_loc[0] - self.tracking_start_loc[0])**2 + (self.source_loc[1] - self.tracking_start_loc[1])**2)

                    drone_dist_from_mline = self.drone_dist_from_mline[:-30]
                    avg_dist_from_mline = int(sum(drone_dist_from_mline) / len(drone_dist_from_mline))
                    max_dist_from_mline = int(max(drone_dist_from_mline))

                    drone_dist_from_closest_contour = self.drone_dist_from_closest_contour[:-30]
                    avg_dist_from_smokecont = int(sum(drone_dist_from_closest_contour) / len(drone_dist_from_closest_contour))
                    max_dist_from_smokecont = int(max(drone_dist_from_closest_contour))
                    
                    drone_within_smoke_contour = self.drone_within_smoke_contour[:-30]
                    percent_time_within_smoke = (sum(drone_within_smoke_contour) / len(drone_within_smoke_contour)) * 100
                    print(drone_within_smoke_contour)
                    print(len(drone_within_smoke_contour))

                    avg_dist_from_mline_norm = (avg_dist_from_mline*100)/tracking_distance
                    max_dist_from_mline_norm = (max_dist_from_mline*100)/tracking_distance

                    avg_dist_from_smokecont_norm = (avg_dist_from_smokecont*100)/tracking_distance
                    max_dist_from_smokecont_norm = (max_dist_from_smokecont*100)/tracking_distance

                    print("\n\n")
                    print("| ------------------------- Tracking Evaluation ------------------------- |")
                    print("|                                                                         |")

                    if avg_dist_from_mline < 10:
                        print(f"|   Average distance of Drone from mean line (of smoke skeleton): {avg_dist_from_mline}       |")
                    elif avg_dist_from_mline >= 10 and avg_dist_from_mline < 100:
                        print(f"|   Average distance of Drone from mean line (of smoke skeleton): {avg_dist_from_mline}      |")
                    else:
                        print(f"|   Average distance of Drone from mean line (of smoke skeleton): {avg_dist_from_mline}     |")

                    if max_dist_from_mline < 10:
                        print(f"|   Maximum distance of Drone from mean line (of smoke skeleton): {max_dist_from_mline}       |")
                    elif max_dist_from_mline >= 10 and max_dist_from_mline < 100:
                        print(f"|   Maximum distance of Drone from mean line (of smoke skeleton): {max_dist_from_mline}      |")
                    else:
                        print(f"|   Maximum distance of Drone from mean line (of smoke skeleton): {max_dist_from_mline}     |")

                    if avg_dist_from_smokecont < 10:
                        print(f"|            Average distance of Drone from smoke (when outside): {avg_dist_from_smokecont}       |")
                    elif avg_dist_from_smokecont >= 10 and avg_dist_from_smokecont < 100:
                        print(f"|            Average distance of Drone from smoke (when outside): {avg_dist_from_smokecont}      |")
                    else:
                        print(f"|            Average distance of Drone from smoke (when outside): {avg_dist_from_smokecont}     |")

                    if max_dist_from_smokecont < 10:
                        print(f"|            Maximum distance of Drone from smoke (when outside): {max_dist_from_smokecont}       |")
                    elif max_dist_from_smokecont >= 10 and max_dist_from_smokecont < 100:
                        print(f"|            Maximum distance of Drone from smoke (when outside): {max_dist_from_smokecont}      |")
                    else:
                        print(f"|            Maximum distance of Drone from smoke (when outside): {max_dist_from_smokecont}     |")

                    print(f"|                         Perecentage of trajectory within smoke: {percent_time_within_smoke:.2f} % |")



                    print(f"|                                        Total Tracking Distance: {int(tracking_distance)}     |")
                    print(f"|   [Normalized] Average distance of Drone from mean line (of smoke skeleton): {avg_dist_from_mline_norm: .2f}       |")
                    print(f"|   [Normalized] Maximum distance of Drone from mean line (of smoke skeleton): {max_dist_from_mline_norm: .2f}       |")
                    print(f"|   [Normalized] Average distance of Drone from smoke (when outside): {avg_dist_from_smokecont_norm: .2f}       |")
                    print(f"|   [Normalized] Maximum distance of Drone from smoke (when outside): {max_dist_from_smokecont_norm: .2f}       |")



                    print("|                                                                         |")
                    print("| ----------------------------------------------------------------------- |")
                    print("\n")

                    break

            except:
                print('Waiting for Proper Data !!!! ', end = '\r')



    def ransac_func(self, x1, x2, ransac_thr, ransac_iter):
        """
        finding the trasformation matrix using RANSAC
        """
        size = x1.shape[0]
        maximum = 0

        for i in range(ransac_iter):
            current = 0
            r = random.sample(range(size), 3)
            sp1 = np.float32(x1[r,:])
            sp2 = np.float32(x2[r,:])

            t_mat = cv2.getAffineTransform(sp1, sp2)
            t_mat = np.append(t_mat, [0,0,1])
            t_mat = t_mat.reshape(3, 3)

            # Compute number of best matches
            for k in range(size):
                p1 = np.append(np.array(x1[k,:]), 1)
                p2 = np.append(np.array(x2[k,:]), 1)
                pt2 = np.matmul(t_mat, p1)
                e = math.dist(pt2, p2)
                if e <  ransac_thr:
                    current += 1

            # best matching F
            if current > maximum:
                maximum = current
                t_mat_final = t_mat

        return t_mat_final



    def coord_global(self, delta_f, theta_f, delta_l, theta_l,phi1, lambda1):
        """
        compute phi2, lambda2
        """
        # Calculate new position for forward/backward movement
        phi2_f = asin(sin(phi1) * cos(delta_f) + cos(phi1) * sin(delta_f) * cos(theta_f))
        lambda2_f = lambda1 + atan2(sin(theta_f) * sin(delta_f) * cos(phi1), cos(delta_f) - sin(phi1) * sin(phi2_f))

        # Calculate new position for lateral movement (combine with forward/backward)
        phi2 = asin(sin(phi2_f) * cos(delta_l) + cos(phi2_f) * sin(delta_l) * cos(theta_l))
        lambda2 = lambda2_f + atan2(sin(theta_l) * sin(delta_l) * cos(phi2_f), cos(delta_l) - sin(phi2_f) * sin(phi2))

        return phi2, lambda2



    def mapping_2d_3d(self, observer_drone_data):
        """
        function for mapping 2D image coordinates to 3D gps coordinates
        returns the transformation matrix T requires for the conversion
        """
        if self.observer_drone_guided_mode:
            phi1 = np.deg2rad(observer_drone_data['lat'])
            lambda1 = np.deg2rad(observer_drone_data['lon'])
            altitude = self.observer_drone_data['Altitude'] - self.observer_drone_data['Altitude_tracker']

            w = (altitude/self.fx)*FRAME_WIDTH
            h = (altitude/self.fy)*FRAME_HEIGHT

            # Forward/backward movement
            delta_f_forward = h/(2*R_EARTH)
            delta_f_backward = -h/(2*R_EARTH)
            theta_f = np.deg2rad(observer_drone_data['heading'])  # forward heading

            # Lateral movement
            delta_l_left = -w/(2*R_EARTH)
            delta_l_right = w/(2*R_EARTH)
            theta_l = theta_f + np.pi / 2  # Add 90 degrees for right, subtract 90 degrees for left

            phi_top_left, lambda_top_left = self.coord_global(delta_f_forward, theta_f , delta_l_left,theta_l, phi1, lambda1)
            phi_top_right, lambda_top_right = self.coord_global(delta_f_forward, theta_f, delta_l_right,theta_l, phi1, lambda1)
            phi_bottom_right, lambda_bottom_right = self.coord_global(delta_f_backward, theta_f, delta_l_right,theta_l, phi1, lambda1)
            phi_bottom_left, lambda_bottom_left = self.coord_global(delta_f_backward, theta_f, delta_l_left,theta_l, phi1, lambda1)

            # Update message(Lat,Long and Alt) and publish
            x1 = np.array([[0,0],[0,640],[480,640],[480,0],[240,320]])
            x2 = np.array([[np.rad2deg(phi_top_left), np.rad2deg(lambda_top_left)],
                           [np.rad2deg(phi_top_right), np.rad2deg(lambda_top_right)],
                           [np.rad2deg(phi_bottom_right), np.rad2deg(lambda_bottom_right)],
                           [np.rad2deg(phi_bottom_left), np.rad2deg(lambda_bottom_left)],
                           [observer_drone_data['lat'], observer_drone_data['lon']]])

            t_mat = self.ransac_func(x1, x2, 3, 500)
            t_mat = np.float32(t_mat)

            return t_mat

        else:
            print("Observer Drone not in guided mode !!!")
            return None

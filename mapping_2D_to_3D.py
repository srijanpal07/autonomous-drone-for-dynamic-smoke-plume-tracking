import argparse
import rospy
from observe_drone_tracking import ObserveDroneTracking



def handle_command_line_arguments():
    """
    function to handle command line arguments
    """
    parser = argparse.ArgumentParser(description="Observer & Tracker Drone command-line parser")

    parser.add_argument("-fx", "--focal_length_x", type=float, default=417.0321044921875, help="Focal Length along X-axis (default: fx = 417.0321044921875)")
    parser.add_argument("-fy", "--focal_length_y", type=float, default=417.0321044921875, help="Focal Length along Y-axis (default: fy = 417.0321044921875)")
    parser.add_argument("observer_drone", nargs="?", default="drone1", help="Observer Drone (default: [\"drone1\"])")
    parser.add_argument("tracker_drone", nargs="?", default="drone2", help="Tracker Drone (default: [\"drone2\"])")

    args = parser.parse_args()

    print(f"fx: {args.focal_length_x}")
    print(f"fy: {args.focal_length_y}")
    print(f"Observer Drone: {args.observer_drone}")
    print(f"Tracker Drone : {args.tracker_drone}")

    return args.focal_length_x, args.focal_length_y, args.observer_drone, args.tracker_drone




if __name__=='__main__':

    fx, fy, observer_drone, tracker_drone = handle_command_line_arguments()
    observe_drone_tracking = ObserveDroneTracking(fx=fx, fy=fy,
                                                     observer_drone=observer_drone,
                                                     tracker_drone=tracker_drone)
    try:
        observe_drone_tracking.show_tracker_drone_in_observer_image()
    except rospy.ROSInterruptException:
        pass

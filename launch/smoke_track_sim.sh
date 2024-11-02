#!/bin/bash

# Open a Terminator terminal
terminator -e "
cd PX4-Autopilot && make px4_sitl_default none_iris; bash" &

terminator -e "
roslaunch mavros px4.launch fcu_url:=udp://:14030@127.0.0.1:14280; bash" &

terminator -e "
rosrun autonomous_drone_for_dynamic_smoke_plume_tracking simCam_node.py; bash" &

terminator -e "
rosrun autonomous_drone_for_dynamic_smoke_plume_tracking detection_node.py; bash" &

terminator -e "
rosrun autonomous_drone_for_dynamic_smoke_plume_tracking segmentation_node.py; bash" &

terminator -e "
rosrun autonomous_drone_for_dynamic_smoke_plume_tracking controller_node.py; bash" &

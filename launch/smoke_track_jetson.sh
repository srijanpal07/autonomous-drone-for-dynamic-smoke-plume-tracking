#!/bin/bash

# Open a Terminator terminal
terminator -e "
roslaunch autonomous_drone_for_dynamic_smoke_plume_tracking mavros-telem-drone.launch; bash" &

terminator -e "
rosrun autonomous_drone_for_dynamic_smoke_plume_tracking simCam_node_1.py; bash" &

terminator -e "
rosrun autonomous_drone_for_dynamic_smoke_plume_tracking segmentation_node.py; bash" &

terminator -e "
rosrun autonomous_drone_for_dynamic_smoke_plume_tracking controller_node.py; bash" &

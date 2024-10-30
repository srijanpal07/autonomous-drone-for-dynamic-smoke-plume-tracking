# Autonomous Drone for Dynamic Smoke Plume Tracking
This repository features the code base for the "Autonomous Drone for Dynamic Smoke Plume Tracking" project. Built on a quadrotor with Nvidia Jetson Orin Nano, it uses vision-based PID and PPO-based DRL controllers for real-time smoke tracking in unsteady wind conditions. The Unreal Engine simulation environment was used to develop and test the vision-based PID control algorithm for smoke plume tracking, as well as to train, test, and refine the deep reinforcement learning-based controller.


## Quick Start

### Clone repo:
```bash
cd ~
mkdir gaia-autonomous-drone
cd gaia-autonomous-drone
mkdir src
cd src
git clone https://github.umn.edu/HongFlowFieldImagingLab/autonomous-drone-for-dynamic-smoke-plume-tracking.git
```

### Run install script:

The install script is configured to install ROS Melodic and the dependencies needed to run yolov8 and Stable Baselines3 PPO.

Have to run this install scripts multiple times after the system automatically gets rebooted untill it shows "Completing Installation of Dependencies ..." on the terminal

```bash
cd ~/gaia-autonomous-drone/src/autonomous-drone-for-dynamic-smoke-plume-tracking/install_scripts
bash install_jetson_dependencies.sh
```

To further optimize RAM space to run deep learning and deep reinforcement learning models, follow the additional steps mentioned in this link - https://www.jetson-ai-lab.com/tips_ram-optimization.html


### Building the ROS Package:
To finalize the installation, initialize and build the ROS package:
```bash
source ~/.bashrc
cd ~/gaia-autonomous-drone
catkin init
catkin build
```

## Running Smoke Tracking Controller:
Quick start added some lines to ~/.bashrc to complete the sourcing of the repo and adding write permissions to the appropriate serial port for communicating with the drone via Mavros (drone and wiring configuration covered in Appendix A). This means the code is ready to run upon opening the terminal and can simply be launched with a single command, e.g.:
```bash
roslaunch GAIA-drone-control track.launch
```
To troubleshoot, run each individual node scripts using rosrun command, each in a separate terminal.

```bash
roslaunch GAIA-drone-control mavros-telem-drone.launch
rosrun GAIA-drone-control cameranode_jetson.py
rosrun GAIA-drone-control feedbackcontrolnode.py
rosrun GAIA-drone-control detectionnode_trt.py
rosrun GAIA-drone-control opticalflownode.py
```

To end collection you can either kill execution in each terminal with Ctrl-C if they are executing locally, or to kill all the terminals altogether or in case if the tasks went to the background (as when starting via ssh then disconnecting) use:

```bash
rosnode kill --all
```


## Configuring the Drone
In general, the drone configuration is very similar to the standard GAIA drone configuration, but with a few channels moved around to make gimbal control accessible to the drone via Mavros since it can only send commands on the first 8 channels.

### Controller Setup

See "ControllerSettings.jpg" in the Setup Resources google drive folder for changes in controller setup. pitch, roll, and yaw on these channel descriptions refer to the gimbal. Channels 1-5 are not shown in the image but simply correspond to the flight control and mode switch channels as before.

https://drive.google.com/drive/folders/1MrDqN7BMAj4Jl0IVvFGdhrfC5YlxuMcb?usp=sharing (NSF-MRI-GAIA/Subproject 1.4 - Drone Feedback Control/Resources/Setup Resources)

### Drone Parameter Setup
There is also a drone parameter file in the same Google Drive folder that has the rc#_option parameters configured to perform the correct actions for the switch assignments. (It is a copy of the parameters from GAIA-4, which was used for this testing.) If using Jetson Xavier, you must also enable all SR0 parameters via Mission Planner config tab. These should all have values set to 10 (hz).

### Telemetry Cable Wiring

For the Jetson Orin Nano, w econnected to Pixhawk via USB telemetry to Jetson USB port (/dev/ttyACM0). To enable communication over Pixhawk USB, you must enable all SR0 parameters via Mission Planner config tab. These should have all the values set to 10 (hz).


## Appendix B: Simulation Environment Setup


# Autonomous Drone for Dynamic Smoke Plume Tracking
This repository features the code base for the "Autonomous Drone for Dynamic Smoke Plume Tracking" project. Built on a quadrotor with Nvidia Jetson Orin Nano, it uses vision-based PID and PPO-based DRL controllers for real-time smoke tracking in unsteady wind conditions. The Unreal Engine simulation environment was used to develop and test the vision-based PID control algorithm for smoke plume tracking, as well as to train, test, and refine the deep reinforcement learning-based controller.


## Quick Start

### Clone repo:
```bash
cd ~
mkdir autonomous-drone
cd autonomous-drone
mkdir src
cd src
git clone https://github.umn.edu/HongFlowFieldImagingLab/autonomous-drone-for-dynamic-smoke-plume-tracking.git
```

### Run install script:

The install script is configured to install ROS Melodic and the dependencies needed to run yolov8 and Stable Baselines3 PPO.

```bash
cd ~/autonomous-drone/src/autonomous-drone-for-dynamic-smoke-plume-tracking/install_scripts
bash install_dependencies.sh
```

### Finalize installation:
To finalize the installation, initialize and build the ROS package:
```bash
source ~/.bashrc
cd ~/autonomous-drone
catkin init
catkin build
```

## Running Feedback Control:
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

Designed to be installed on a Jetson mini running Ubuntu 18.04. Can also be installed on WSL or Virtualbox.

### Step 1: Simulation Environment Setup:
This was mostly accomplished by quick start. 
It can likely be completed simply by running the following:
```bash
sudo apt install ros-melodic-desktop
```


### Step 2: Arducopter SITL installation
Overarching tutorial:
https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
From there first follow link to:
https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

#### Clone repo, checkout 4.0.7 release
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout 0bb18a153c
git submodule update --init --recursive
```
#### Install requirements
Use tools script included with the repo
```bash
cd ~/arducopter
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Reload the path (will become permanent when device is restarted)
```bash
. ~/.profile
```
#### Build Arducopter
Instructions at https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
Use board type 'sitl' (software in the loop), which is also the default
```bash
./waf configure --board sitl
./waf copter
```
From here the build is complete and should be ready to follow the demo tutorials above.

## Appendix C: Simulator Demonstrations


### Basic Arducopter SITL Startup Example w/ Mavros telemetry

In one terminal:
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```
In a second terminal:
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```

Launch a mavros instance in a third terminal:
```bash
cd ~/catkin_ws/src/GAIA-drone-control/launch
roslaunch mavros-telem.launch
```

The mavros telemetry can then be viewed in a fourth terminal. First verify publishing rate with rostopic hz, then see contents with echo:
```bash
rostopic hz /mavros/local_position/pose
ctrl-c
rostopic echo /mavros/local_position/pose
```

This completes the drone startup,
\
### Manual Drone Control Using Arducopter SITL Terminal
The following commands can be entered in the arducopter terminal (second terminal from instructions above) to control the vehicle:
#### Takeoff:
```bash
GUIDED
arm throttle
takeoff 10
```
This places the drone into guided mode (so it can accept computer control), arms it, and tells it to take off to 10m. 
\
\
After takeoff the "position x y z" or "velocity vx vy vz" commands can be used to move the drone. For position x,y, and z are relative distances to move  and in north, east, down coordinates, e.g. position 1 0 -1 would move 1m forward and 1m up. The "help" command lists these and other commands, and typing most commands such as "position" or "velocity" without arguments provides some basic instructions on how to use them.

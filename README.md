# Autonomous Drone for Dynamic Smoke Plume Tracking
This repository features the code base for the "Autonomous Drone for Dynamic Smoke Plume Tracking" project. Built on a quadrotor with Nvidia Jetson Orin Nano, it uses PID and PPO-based DRL controllers for real-time smoke tracking in unsteady wind conditions. Tested in Unreal Engine simulations and field tests, it has applications in wildfire management and air quality monitoring.


## Quick Start

### Clone repo:
```bash
cd ~
mkdir gaia-feedback-control
cd gaia-feedback-control
mkdir src
cd src
git clone https://github.umn.edu/HongFlowFieldImagingLab/GAIA-drone-control.git
```

### Run install script: (This will take quite some time on the Jetson and you will likely be prompted to re-enter the root password a few times)

```bash
cd ~/gaia-feedback-control/src/GAIA-drone-control/install_scripts
bash install_dependencies.sh
```
The install script is configured to install ROS and the dependencies needed to run yolov5. 

<!-- It does not currently install Spinnaker or PySpin, this must be done manually to complete the setup and be able to run things. 

### Manually install spinnaker node, spinnaker python:
Either download directly from spinnaker website, or from our Google Drive:

https://drive.google.com/drive/folders/1MrDqN7BMAj4Jl0IVvFGdhrfC5YlxuMcb?usp=sharing (NSF-MRI-GAIA/Subproject 1.4 - Drone Feedback Control/Resources/Setup Resources)

spinnaker_python-2.5.0.80-cp36-cp36m-linux_aarch64.tar.gz and spinnaker_python-2.5.0.80-cp36-cp36m-linux_aarch64.tar.gz

Direct download: https://flir.app.boxcn.net/v/SpinnakerSDK/folder/74729115388

Install standard spinnaker SDK first, then spinnaker-python following the readme's from each zip folder.

In general, to install spinnaker_python-2.5.0.80-cp36-cp36m-linux_aarch64 unzip and call:
```bash
sudo sh install_spinnaker_arm.sh
```
Accept the agreement and answer yes to all the prompts (except probably making system examples available in path, flir GenTL Producer, and the feedback program). Enter the jetson username (usually jetson) when prompted for a username, then enter to skip when prompted for another.

To install spinnaker_python-2.5.0.80-cp36-cp36m-linux_aarch64 unzip and run:
```bash
sudo pip3 install spinnaker_python-2.5.0.80-cp36-cp36m-linux_aarch64.whl
``` -->
### Finalize installation:
To finalize the installation, initialize and build the catkin project to make finding ROS packages easy:
```bash
source ~/.bashrc
cd ~/gaia-feedback-control
catkin init
catkin build
```

You MUST restart the device before spinnaker will work, but then the Jetson should be ready to run any of the ROS and yolo_v5 code used for feedback control.

## Running Feedback Control:
Quick start added some lines to ~/.bashrc to complete the sourcing of the repo and adding write permissions to the appropriate serial port for communicating with the drone via Mavros (drone and wiring configuration covered in Appendix A). This means the code is ready to run upon opening the terminal and can simply be launched with a single command, e.g.:
```bash
roslaunch GAIA-drone-control track.launch
```
To troubleshoot, single nodes can be run by launching a roscore in one terminal and running the individual node scripts with rosrun, each in a separate terminal. Each node simply runs like a python script and can be terminated, editted, and restarted without interrupting the others (apart from pausing the stream of published/subscribed data).
```bash
roscore
roslaunch GAIA-drone-control mavros-telem-drone.launch
rosrun GAIA-drone-control cameranode.py
rosrun GAIA-drone-control feedbackcontrolnode.py
rosrun GAIA-drone-control detectionnode_trt.py
rosrun GAIA-drone-control opticalflownode.py
```


To end collection you can either kill execution in each terminal with Ctrl-C if they are executing locally, or if the tasks went to the background (as when starting via ssh then disconnecting during a field deployment) use:
```bash
rosnode kill --all
```
### Recording data

Each node is set up to automatically create new directories and run numbers based on the date, with video files and meta data recorded continuously. This allows the most simple analysis of the data collected.

Alternatively, you can record a rosbag. Recording a rosbag of the data produced by the nodes (including image data) can be done with the following: (records to current directory, very large files)
```bash
rosbag record -a --split --duration=60
```


## Appendix A: Configuring the Drone
In general, the drone configuration is very similar to the standard GAIA drone configuration, but with a few channels moved around to make gimbal control accessible to the drone via Mavros since it can only send commands on the first 8 channels.

### Controller Setup

See "ControllerSettings.jpg" in the Setup Resources google drive folder for changes in controller setup. pitch, roll, and yaw on these channel descriptions refer to the gimbal. Channels 1-5 are not shown in the image but simply correspond to the flight control and mode switch channels as before.

https://drive.google.com/drive/folders/1MrDqN7BMAj4Jl0IVvFGdhrfC5YlxuMcb?usp=sharing (NSF-MRI-GAIA/Subproject 1.4 - Drone Feedback Control/Resources/Setup Resources)

### Drone Parameter Setup
There is also a drone parameter file in the same Google Drive folder that has the rc#_option parameters configured to perform the correct actions for the switch assignments. (It is a copy of the parameters from GAIA-4, which was used for this testing.) If using Jetson Xavier, you must also enable all SR0 parameters via Mission Planner config tab. These should all have values set to 10 (hz).

### Gimbal Wiring/Setup

The gimbal pitch, roll, yaw channels are wired to Pixhawk output channels 5, 6, and 7. This is the same wire bundle the ESC signal wires connect to. The ground pin from the gimbal must be connected to one of the ground pins, otherwise the signal pin for each channel just needs to be connected to the signal pin.

There is a gimbal setup GUI in Mission Planner under Setup/Optional/Camera Gimbal. See the screenshot in Google Drive for the settings used on GAIA-4.

### Telemetry Cable Wiring

For the Jetson Xavier, the only way to connect is via Pixhawk USB telemetry to Jetson USB port (/dev/ttyACM0). UART2 for unknown reason allows for write from Jetson, but not read from Pixhawk. However, to enable communication over Pixhawk USB, you must enable all SR0 parameters via Mission Planner config tab. These should all have values set to 10 (hz).

For the Jetson Nano, this issue does not come up. The telemetry cable is wired to connect the Jetson Nano's UART_2 (/dev/ttyTHS1) to the Pixhawk Telemetry 2 port. This telemetry port is enabled already.

The following link was used to identify the correct pins and create wiring for connecting to the Telemetry 2 port. The extra cables that come with telemetry radios are excellent sources of the appropriate connector, otherwise you'll just need to order some telemetry cables.

https://www.jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/

| Pixhawk Telem Pin |   Jetson Pin      |
| -----------       |   ---------       |
|   1 (red, 5V)     |   NA              |
|   2 TX            |   10 (UART_2_RX)  |
|   3 RX            |   8 (UART_2_TX)   |
|   4 CTS           |   11 (UART_2_RTS) |
|   5 RTS           |   36 (UART_2_CTS) |
|   6 GND           |   6 (GND)         |

If using the Jetson Xavier, connect from the Pixhawk's USB telemetry to a standard USB port on the Jetson Xavier.


## Appendix B: Simulation Environment Setup

Designed to be installed on a Jetson mini running Ubuntu 18.04. Can also be installed on WSL or Virtualbox.

### Step 1: Ros Melodic/Mavros/Gazebo Installation:
This was mostly accomplished by quick start. 
It can likely be completed simply by running the following: (Untested)
```bash
sudo apt install ros-melodic-desktop
```

Otherwise, to reinstall ROS with the complete desktop simulators follow the install instructions for ROS Melodic on the ROS wiki. (Or look up the apt commands for gazebo, rviz, and rqt for ROS Melodic) 
http://wiki.ros.org/melodic/Installation/Ubuntu

Choose 'sudo apt install ros-melodic-desktop' when choosing how many of the resources to install, this includes the necessary and useful simulation packages without using as much storage space as the desktop-full option.

After completing these steps ensure the 'catkin' command is recognized. If it is not, install with the following command:
```bash
sudo apt-get install python3-catkin-tools
```

Install necessary computer vision tools:
```bash
sudo apt-get install ros-melodic-vision-msgs ros-melodic-vision-opencv ros-melodic-cv-bridge
```

Install mavros using the following command:
```bash
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
```
Install Gazebo:
```bash
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
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

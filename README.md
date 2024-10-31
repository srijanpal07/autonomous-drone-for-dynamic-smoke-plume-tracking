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

The install script is configured to install ROS Melodic and necessary dependencies for running YOLOv8 and Stable Baselines3 PPO.

May need to run the installation script multiple times after the system automatically reboots until you see the message `Completing Installation of Dependencies ...` in the terminal. To execute the script, use the following commands:
```bash
cd ~/gaia-autonomous-drone/src/autonomous-drone-for-dynamic-smoke-plume-tracking/install_scripts
bash install_jetson_dependencies.sh
```

To further optimize RAM for running deep learning and deep reinforcement learning models on Jetson, refer to the additional steps outlined in this [link](https://www.jetson-ai-lab.com/tips_ram-optimization.html)


### Building the ROS Package:
To finalize the installation, initialize and build the ROS package:
```bash
source ~/.bashrc
cd ~/gaia-autonomous-drone
catkin init
catkin build
echo "source ~/gaia-autonomous-drone/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Running Smoke Tracking Controller:
The "Quick Start" setup automatically adds necessary lines to `~/.bashrc` to source the repository and grant write permissions to the required serial port for communicating with the Pixhawk through MAVROS. With this configuration, the code is ready to run as soon as a terminal opens. 

### Starting the controller:
To initiate smoke tracking, use the following command:
```bash
roslaunch autonomous_drone_for_dynamic_smoke_plume_tracking smoke_track_jetson.launch execution:=DEPLOY
```

### Configurable parameters:
Three parameters can be specified when launching the controller to tailor the setup to specific needs:

* **'drone'** : Specifies the MAVROS namespace of the drone. 
    * **Default**: `drone1` 
    * **Usage**: Use `drone:=<namespace>` if the namespace is different from `drone1`.

[By default, the namespace of the drone is mentioned as 'drone1', specify the namespace of the drone if it is something else.]

'execution' ->  Option ['SIM' or 'DEPLOY'] to choose between code execution is done in simulation or in Jetson for deployment. [By default, the execution is set to 'SIM' (Simulation). To switch to 'DEPLOY' (Deployment in Jetson), specify the execution type as an argument.]


'controller' -> Option ['PID' or 'DRL'] to choose between controller type to use [By default, the controller node operates with the PID (Proportional–Integral–Derivative) controller. To switch to the DRL (Deep Reinforcement Learning) controller, specify the controller type as an argument.]

Example - 

For executing in Jetson on 'drone1' with PID controller:
```bash
roslaunch autonomous_drone_for_dynamic_smoke_plume_tracking smoke_track_jetson.launch drone:=drone1 execution:=DEPLOY controller:=PID
```
For executing in Jetson on 'drone2' with DRL controller:
```bash
roslaunch autonomous_drone_for_dynamic_smoke_plume_tracking smoke_track_jetson.launch drone:=drone2 execution:=DEPLOY controller:=DRL
```

To troubleshoot, run this bash script :
```bash
cd ~/gaia-autonomous-drone//src/autonomous-drone-for-dynamic-smoke-plume-tracking/launch
chmod +x smoke_track_jetson.sh
./smoke_track_jetson.sh
```
This should run all the nodes in seprate terminals.

To stop execution, you can either press Ctrl-C in each terminal or, to terminate all nodes at once (useful if they’ve gone to the background, such as when started over SSH), run:
```bash
rosnode kill --all
```
This command will safely end all running ROS nodes.


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


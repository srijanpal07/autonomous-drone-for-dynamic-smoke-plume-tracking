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

The install script is configured to install ROS Melodic and necessary dependencies for running YOLOv8 and Stable Baselines3 PPO. May need to run the installation script multiple times after the system automatically reboots until you see the message `Completing Installation of Dependencies ...` in the terminal. 

To execute the script, use the following commands:
```bash
cd ~/gaia-autonomous-drone/src/autonomous-drone-for-dynamic-smoke-plume-tracking/install_scripts
bash install_jetson_dependencies.sh
```

To further optimize RAM for running deep learning and deep reinforcement learning models on Jetson, refer to the additional steps outlined in this [link](https://www.jetson-ai-lab.com/tips_ram-optimization.html) .


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

* **'execution'** : Specifies the execution environment.
    * **Options**: `SIM` (Simulation) | `DEPLOY` (Jetson Deployment)
    * **Default**: `SIM` 
    * **Usage**: Set `execution:=DEPLOY` for real-world deployment on Jetson.

* **'controller'** : Selects the controller type.
    * **Options**: `PID` (Proportional–Integral–Derivative Controller) | `DRL` (Deep Reinforcement Learning Controller)
    * **Default**: `PID` 
    * **Usage**: Set `controller:=DRL` to use the DRL-based controller.

Example Commands - 

For **executing in Jetson** on `drone1` with `PID` controller:
```bash
roslaunch autonomous_drone_for_dynamic_smoke_plume_tracking smoke_track_jetson.launch drone:=drone1 execution:=DEPLOY controller:=PID
```
For **executing in Jetson** on `drone2` with `DRL` controller:
```bash
roslaunch autonomous_drone_for_dynamic_smoke_plume_tracking smoke_track_jetson.launch drone:=drone2 execution:=DEPLOY controller:=DRL
```

### Troubleshoot:
Run this bash script :
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


## Drone Hardware Configuartions

This drone configuration closely follows the standard [GAIA drone setup](https://github.umn.edu/HongFlowFieldImagingLab/GAIA-drone-control/tree/peter-server) and is based on the [Holybro S500 v2 development kit](ttps://holybro.com/collections/s500/products/s500-v2-development-kit). The following upgrades enhance real-time performance and adaptability for dynamic smoke tracking:

### Hardware Upgrades:

1. **Jetson Orin Nano (Primary Edge Computing Board)**
	* **Upgrade:** The Jetson Orin Nano replaces the Jetson Xavier as the primary onboard computer.
	* **Recommendation:** Booting from an NVMe SSD is highly recommended to improve inference speeds for deep learning and deep reinforcement learning models in real-time. Details on setting up NVMe SSD boot can be found [here](https://forums.developer.nvidia.com/t/jetson-orin-nano-boot-from-nvme-ssd/252492).

2. **12 MP USB ArduCam (Primary Camera)**
	* **Upgrade:** The ArduCam has replaced the GoPro, providing higher resolution images faster in real-time.

3. **Updated Pixhawk Parameters**
	* **Upgrade:** Configured Pixhwak parameters for optimized drone performance - [link-to-parameter-list]().

4. **Custom 3D-Printed Enclosure and Battery Holder**
	* **Upgrade:** Custom-designed enclosure and battery holder for compactness and safety during operation.


## Unreal Engine 5 Simulation Environment Setup


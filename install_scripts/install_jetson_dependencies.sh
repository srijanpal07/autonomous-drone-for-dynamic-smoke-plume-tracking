# Install dependencies for autonomous-drone-for-dynamic-smoke-plume-tracking
# Run './install_dependencies.sh' in terminal (without sudo)
# Have to run this install scripts multiple times after the system automatically gets rebooted 
# Untill it shows "Completing Installation of Dependencies ..." on the terminal


# Define the checkpoint file path
CHECKPOINT_FILE="/tmp/install_checkpoint.txt"

# Function to set a checkpoint
set_checkpoint() {
    echo "$1" > "$CHECKPOINT_FILE"
}

# Function to get the current checkpoint
get_checkpoint() {
    if [ -f "$CHECKPOINT_FILE" ]; then
        cat "$CHECKPOINT_FILE"
    else
        echo "START"
    fi
}



# Install all python modules to virtual environment
# Optionally you can create your own environment and install dependencies
py_env=/usr/bin/python3.8

# Step 1: Update package list and install pip
if [ "$(get_checkpoint)" == "START" ]; then
    echo "Updating package list and installing pip..."
    $py_env -m pip install --upgrade pip

    sudo apt update -y
    sudo apt-get update -y
    
    # Install ninja for faster compile for source
    sudo apt-get install -y ninja-build
    $py_env -m pip install ninja
    
    set_checkpoint "STEP_1_DONE"
fi


# ========================================================== Install ROS Noetic ========================================================== #

# Instructions from https://wiki.ros.org/noetic/Installation/Ubuntu
# Step 2: ROS Installation
if [ "$(get_checkpoint)" == "STEP_1_DONE" ]; then
    echo "Installing ROS Noetic ..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    
    # Conditionally add lines to bashrc if it is not already there
    if ! grep -Fxq "source /opt/ros/noetic/setup.bash" ~/.bashrc; 
        then echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc; 
    fi

    # Convenience source for the gaia-autonomous-drone-ws, if you are working with other workspaces as well you may want to drop this
    # if ! grep -Fxq "source ~/gaia-autonomous-drone/devel/setup.bash" ~/.bashrc; 
    #    then echo "source ~/gaia-autonomous-drone/devel/setup.bash" >> ~/.bashrc; 
    # fi

    # Convenience call to change permission on /dev/ttyACM0 so this command does not have to be run before launching Mavros
    if ! grep -Fxq "sudo chmod 777 /dev/ttyACM0" ~/.bashrc; 
        then echo "sudo chmod 777 /dev/ttyACM0" >> ~/.bashrc; 
    fi

    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update

    # Warning: running 'rosdep update' as root is not recommended.
    # Should run 'sudo rosdep fix-permissions' and invoke 'rosdep update' again without sudo to fix it 
    # if you accidentally execute this install script with sudo.

    sudo apt-get install -y python3-catkin-tools

    sudo bash install_geographiclib_datasets.sh
    
    set_checkpoint "STEP_2_DONE"
fi

# ========================================================== End of ROS Noetic Installation ========================================================== #



# Step 3: Install Vision Tools, Mavros and catkin pkgs
if [ "$(get_checkpoint)" == "STEP_2_DONE" ]; then
    echo "Installing Vision Tools, Mavros and catkin pkgs"
    # Install some necessary ros vision tools
    sudo apt-get install -y ros-noetic-vision-msgs ros-noetic-vision-opencv ros-noetic-cv-bridge

    # Install Mavros
    sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
    sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

    # Install some fixes for using ros noetic with Python3
    sudo apt-get install -y python3-pip python3-yaml
    $py_env -m pip install rospkg catkin_pkg

    # Install necessary python packages
    $py_env -m pip install Cython
    $py_env -m pip install numpy

    sudo apt-get install -y gfortran libopenblas-dev liblapack-dev
    
    set_checkpoint "STEP_3_DONE"
fi



# ========================================================== Install Dependencies for Yolov8 ========================================================== #

# Instructions from https://docs.ultralytics.com/guides/nvidia-jetson/?h=nvidia#install-onnxruntime-gpu

# Step 4: Update package list and install pip
if [ "$(get_checkpoint)" == "STEP_3_DONE" ]; then
    echo "Installing Dependencies for YOLOv8 ..."
    # Install Ultralytics Package
    # Update packages list, install pip and upgrade to latest
    sudo apt update
    sudo apt install python3-pip -y
    $py_env -m pip install -U pip

    # Install ultralytics pip package with optional dependencies
    $py_env -m pip install ultralytics[export]

    set_checkpoint "REBOOT_1_REQUIRED"
fi

if [ "$(get_checkpoint)" == "REBOOT_1_REQUIRED" ]; then
    echo "Rebooting System ..."
    set_checkpoint "REBOOT_1_DONE"
    # This will reboot the device
    sudo reboot
fi

if [ "$(get_checkpoint)" == "REBOOT_1_DONE" ]; then
    echo "Continuing with YOLOv8 Dependencies Installations ..."
    # Install PyTorch and Torchvision
    # Uninstall currently installed PyTorch and Torchvision
    $py_env -m pip uninstall torch torchvision

    # Install PyTorch 2.1.0 according to JP5.1.3
    sudo apt-get install -y libopenblas-base libopenmpi-dev
    wget https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl -O torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
    $py_env -m pip install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

    # Install Torchvision v0.16.2 according to PyTorch v2.1.0
    sudo apt install -y libjpeg-dev zlib1g-dev
    cd ~
    git clone https://github.com/pytorch/vision torchvision
    cd torchvision
    git checkout v0.16.2
    python3.8 setup.py install --user

    # Install onnxruntime-gpu
    wget https://nvidia.box.com/shared/static/zostg6agm00fb6t5uisw51qi6kpcuwzd.whl -O onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl
    $py_env -m pip install onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl

    # onnxruntime-gpu will automatically revert back the numpy version to latest. So we need to reinstall numpy to 1.23.5 to fix an issue by executing:
    $py_env -m pip install numpy==1.23.5
    
    set_checkpoint "STEP_4_DONE"
fi

# ========================================================== End of Dependencies for Yolov8 Installation ========================================================== #



# Step 5: Install additional dependencies
if [ "$(get_checkpoint)" == "STEP_4_DONE" ]; then
    echo "Installing Additional Dependencies..."
    $py_env -m pip install opencv-python

    $py_env -m pip install requests
    $py_env -m pip install --upgrade scipy
    $py_env -m pip install tqdm
    $py_env -m pip install tensorboard
    $py_env -m pip install pandas
    $py_env -m pip install seaborn
    $py_env -m pip install thop
    
    $py_env -m pip install stable-baselines3[extra]
    $py_env -m pip install gymnasium

    sudo apt install terminator

    set_checkpoint "STEP_5_DONE"
fi



# ============================================== To enable maximum performance on the NVIDIA Jetson for running YOLOv8 ============================================== #

# Step 6: Enabling maximum power in Jetson
if [ "$(get_checkpoint)" == "STEP_5_DONE" ]; then
    echo "Enabling maximum power in Jetson..."
    # Enable MAX Power Mode
    sudo nvpmodel -m 0

    # Enable Jetson Clocks
    sudo jetson_clocks

    # Install Jetson Stats application
    sudo apt update
    sudo pip install jetson-stats

    set_checkpoint "REBOOT_2_REQUIRED"
fi

if [ "$(get_checkpoint)" == "REBOOT_2_REQUIRED" ]; then
    echo "Rebooting System ..."
    set_checkpoint "REBOOT_2_DONE"
    # This will reboot the device
    sudo reboot
fi

if [ "$(get_checkpoint)" == "REBOOT_2_DONE" ]; then
    echo "Completing Installation of Dependencies ..."
    # To check CPU, GPU, RAM utilization, change power modes, set to max clocks:
    # Run 'jtop' in terminal
    set_checkpoint "STEP_6_DONE"
fi

# ============================================== Completed enabling maximum performance on the NVIDIA Jetson ============================================== #




# ============================================== Mounting Swap ============================================== #

# Instructions - https://www.jetson-ai-lab.com/tips_ram-optimization.html
# It's advisable to mount SWAP (typically correlated with the amount of memory in the board). Run these commands to disable ZRAM and create a swap file:
# If you have NVMe SSD storage available, it's preferred to allocate the swap file on the NVMe SSD.

sudo systemctl disable nvzramconfig
sudo fallocate -l 16G /ssd/16GB.swap
sudo mkswap /ssd/16GB.swap
sudo swapon /ssd/16GB.swap

# Then add the following line to the end of /etc/fstab to make the change persistent:
/ssd/16GB.swap  none  swap  sw 0  0

# ============================================== Completed Mounting Swap ============================================== #



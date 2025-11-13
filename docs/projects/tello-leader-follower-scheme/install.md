icon:fontawesome/solid/gear

This sections provides comprehensive instructions for installing and configuring the **DJI Tello leader follower scheme** control system, it covers dependency installation, ROS 2 Iron setup, Hailo AI module integration and environment configuration. The guide supports **Raspberry Pi OS** on **Raspberry Pi 5** as the primary platform. 

## System Requirements

### Hardware Requirements
* Raspberry Pi 5 with 8GB RAM
* Storage: 32 GB microSD card (Class 10)
* DJI Tello Drone
* Hailo-8 AI module

Software Requirements
* Operating System: Raspberry OS (64-bit, ARM)
* Python: 3.11
* ROS 2: Iron distribution
* Additional: Git, pip, Python virtual environment

## Environment Setup: 
### Update the system
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git python3-pip
```
### ROS 2 Iron Installation on Raspberry OS

Install: 
```bash
wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-iron-desktop-0.3.2_20231028_arm64.deb
sudo apt install ./ros-iron-desktop-0.3.2_20231028_arm64.deb
sudo pip install --break-system-packages vcstool colcon-common-extensions
```

Load ROS2: 
```bash
source /opt/ros/${DISTRO}/setup.bash
# e.g. source /opt/ros/iron/setup.bash
```

!!! quote "Quote"
    Check the repository content for more information:[ROS 2 Iron Installation](https://github.com/Ar-Ray-code/rpi-bullseye-ros2/tree/iron?tab=readme-ov-file "ros2-iron")

### cvBridge installation

Install numpy and libboost
```bash
sudo apt install python3-numpy
sudo apt install libboost-python-dev
```
Create a ROS 2 Workspace:
```bash
mkdir -p ~/vision_opencv/src
```
open the src folder: 
```bash
cd vision_opencv/src
```
clone the repository in the iron branch:
```bash
git clone https://github.com/ros-perception/vision_opencv.git -b iron
``` 
Compile the package: 
```bash
cd ../
colcon build --symlink_install
```
activate the environment you just compiled: 
```bash
source install/setup.bash
```
!!! quote "Quote"
    Check the repository to get more information: [Vision opencv](https://github.com/ros-perception/vision_opencv/tree/iron?tab=License-2-ov-file "vision-opencv")

### Install the ROS 2 interfaces
create a ROS 2 Workspace and open your src file: 
```bash
mkdir -p ~/drone_det_itfc/src
cd ~/drone_det_itfc/src
```
clone the repository: 
```bash
git clone https://github.com/LichtenbergCode/drone-tello-leader-follower-interfaces.git 
```
Compile the package:
```bash
cd ../
colcon build
```
activate the environment you just compiled: 
```bash
source install/setup.bash
```

??? tip "Activate your environments since your .bashrc file"
    .bashrc is a script that runs automatically and configures your workspace when you open the terminal. By activating your workspaces within the .bashrc file, you don't need to activate them manually as before. 
    
    Open your ```.bashrc``` file:
    ```bash
    gedit .bashrc
    ```
    At the end of the file add the workspace activations: 
    ```bash
    source vision_opencv/install/setup.bash
    source drone_det_itfc/install/setup.bash
    ```

### Create a Python Virtual environment and the Workspace of the project
Create The workspace: 
```bash
```

### Hailo-8 AI Acceletor module Integration

#### Hailo-8 Overview
<img src="../img/installation/hailo8.png" alt="Hailo-8 accelerator" align="right" width=40%>

The Hailo-8 is specialized AI processor designed for high-performance, low-power edge computing applications. With up to 26 TOPS (Tera Operatios Per Second) while consuming only 2.5W of power, it provides exceptional efficiency for real-time AI inference tasks on embedded systems like the Raspberry Pi 5. 

#### Installing firmware and Python pipelines

update and upgrade the system: 
```bash
sudo apt update && sudo apt full upgrade
```
Install the complete Hailo AI software suite:
```bash
sudo apt install install hailo-all
```
Restart your Pi: 
```bash
reboot
```
Install ```hailo-apps-infra```:   
```bash
# Clone the repository
git clone https://github.com/hailo-ai/hailo-apps-infra.git
cd hailo-apps-infra

# Run the installer
./install.sh
```
Setup environment: 
```bash
source setup_env.sh
```
!!! quote "Quote"
    Check the repository content for more information: 
    [Hailo Apps Infra](https://github.com/hailo-ai/hailo-apps-infra?tab=readme-ov-file "hailo-apps-infra")

#### Modifying **setup_env.sh**
copy the path to of your library python packages:
```bash
cd ~/hailo-apps-infra/venv_hailo_apps/lib/python3.11/site-packages 
pwd # To watch your path 
``` 
Open your setup_env.sh file: 
```bash
gedit setup_env.sh
```
Paste your python path with the next structure: 
```bash
export PYTHONPATH=${PYTHONPATH}:home/<user>/<path-to-your-hailo-environment>/venv_hailo_apps/lib/python3.11/site-packages
```

## git clone and creating your python venv

### Creating a Python virtual environment 
Create a new workspace:
```bash
cd ~
mkdir my_ros_ws
```
open the directory: 
```bash
cd my_ros_ws
```
To create the virtual environment, run: 
```bash
virtualenv -p python3 ./venv
```
Activate the virtual environment using the next source command: 
```bash
source ./venv/bin/activate
```
After you activate the environment your command prompt will change to show that you are in a Python virtual environment.

To use a virtual environment with ROS 2 you must tell colcon to ignore the directory that contains the files that manage the virtual environment: 
```bash
touch ./venv/COLCON_IGNORE
```
To exit your virtual virtual environment type: 
```bash
deactivate
```

To ensure your ROS 2 distribution can access Python packages installed in your virtual environment, you need to explicitly add the virtual environment's Python path to ROS's Python system path. This bridges the gap between your isolated Python environment and the ROS 2 ecosystem.

Add the following lines to your virtual environment's activation script to automatically configure the Python path when the environment is activated:
```bash
export PYTHONPATH=${PYTHONPATH}:/path-to-your-env/lib/python3.11/site-packages
```
### Project Cloning and Initialization
clone the repository into your drone_altitude_ws workspace: 
```bash
git clone https://github.com/LichtenbergCode/tello-pid-altitude-control-ros2.git
```
Activate your Python virtual environment: 
```bash
source venv/bin/activate
```
Install the necessary Python modules for the application: 
```bash
pip install -r requirements -v
```
Compile the packages: 
```bash
colcon build
```
!!! warning "Use ```colcon build``` only within your my_ros_ws workspace" 
Activate the environment you just compiled: 
```bash
source install/setup.bash
```

create and open a **ROS 2 workspace** into your hailo workspace:
```bash
cd hailo-apps-infra
mkdir ros_dt_ws
cd ros_dt_ws
```
clone the repository: 
```bash
git clone https://github.com/LichtenbergCode/drone-tello-leader-follower-vision.git
```
compile the package: 
```bash
colcon build
```
Activate the environment you just compiled: 
```bash
source install/setup.bash
```
To run check the next section: [User guide](user-guide.md "user-guide") 

## Troubleshooting 
### Tello Connection Problems
!!! failure "**Issue: Cannot connect ro Tello drone**"
    Check WiFi connection: 
    ```bash
    nmcli device wifi list
    nmcli connection show
    ```
    Verify Tello WiFi is available:
    ```bash
    sudo iwlist scan | grep -i tello
    ```
    Restart network manager:
    ```bash
    sudo systemctl restart NetworkManager
    ```
### 4.2 Module problems
!!! bug "**Issue: "ModuleNotFoundError: No module named 'matplotlib.tri.triangulation'"**" 
    Uninstall matplotlib in your virtual environment: 
    ```bash
    pip uninstall matplotlib -y
    ``
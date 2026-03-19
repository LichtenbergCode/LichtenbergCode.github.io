icon:fontawesome/solid/gear

This sections provides comprehensive instructions for installing and configuring the **DJI Tello leader follower scheme** control system, it covers dependency installation, ROS 2 Iron setup, Hailo AI module integration and environment configuration. The guide supports **Raspberry Pi OS** on **Raspberry Pi 5** as the primary platform. 

## System Requirements

### Hardware Requirements
* Raspberry Pi 5 with 8GB RAM
* Storage: 32 GB microSD card (Class 10)
* DJI Tello Drone
* Hailo-8 AI module

Software Requirements
* Operating System: Ubuntu 24.04 (64-bit, ARM)
* Additional: Git, pip, Python virtual environment

## Environment Setup: 

### Hailo Drivers Installation 

<img src="../img/installation/hailo8.png" alt="Hailo-8 accelerator" align="right" width=40%>

The Hailo-8 is specialized AI processor designed for high-performance, low-power edge computing applications. With up to 26 TOPS (Tera Operatios Per Second) while consuming only 2.5W of power, it provides exceptional efficiency for real-time AI inference tasks on embedded systems like the Raspberry Pi 5. 

**update the system**
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git python3-pip
```

**Installing Required Dependencies**
```bash
sudo apt install -y build-essential cmake git unzip wget curl pkg-config ca-certificates gnupg software-properties-common

sudo apt-get install build-essential dkms linux-headers-$(uname -r)
```
Install al the necessary tools to build software on your system. This includes compilers, development utilities, and kernel headers, which are required to compile and install the drivers. 


**Cloning the Driver Repository**
```bash
git clone https://github.com/hailo-ai/hailort-drivers.git
```
Download the official Hailo driver repository from GitHub to your local machine.


**Selecting the Compatible Version**
```bash
cd hailort-drivers
git checkout v4.20.0
``` 
Navigates into the repository and switches to a specific driver version (v4.20.0) to ensure compatibility with the SDK and hardware. 


**Building and Install the Driver**
```bash 
cd linux/pcie/
make all 
sudo make install 
```
Compiles the driver from source code and installs it into the system. This step creates the necessary kernel modules for communicating with the Hailo device. 

**Loading the Kernrl Module**
```bash
sudo modprobe hailo_pci
```
Manually loads the driver module into the kernel, allowing the system to recognize the Hailo device without requiring a reboot

**Returning to the Root Directory**
Go to hailort-drivers file: 
```bash
cd ../..
```
Return to the root directory of the repository to continue the setup process. 

**Installing Firmware and Udev Rules**
```bash
sudo mkdir -p /lib/firmware/hailo
./download_firmware.sh
sudo mv hailo8_fw.4.20.0.bin /lib/firmware/hailo/hailo8_fw.bin
sudo cp ./linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger 
```
Creates the firmware directory, downloads the required firmware, and moves it to the correct location. It also installs udev rules so the system can automatically manage device permission, then reloads those rules. 


**Rebooting the System**
```bash
reboot
```
Restarts the system to ensure all changes (driver, firmware, and rules) are properly applied


**Verify the Installation**
```bash
sudo dmesg | grep hailo 
```
Check that the driver was successfully loaded by inspecting kernel messages. If everything is working correctly, you should see entries related to the Hailo device. 

### HailoRT Installation 


**Cloning the HailoRT Repository**
```bash 
git clone https://github.com/hailo-ai/hailort.git
cd hailort
git checkout v4.20.0
```
Clones the official HailoRT repository from GitHub, enters the project directory, and checks out a specific version (v4.20.0) to ensure compatibility with the installed drivers.


**Building and Installing HailoRT**
```bash
cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release -DHAILO_BUILD_EXAMPLES=1 && sudo cmake --build build --config release --target install
```
Configures, compilers, and installs the HailoRT library using CMake. 

* ```-DCMAKE_BUILD_TYPE=Release``` enables optimized compilation

* ```-DHAILO_BUILD_EXAMPLES=1```includes example applications

* The build in then installed system-wide

**Verifying Installation**
```bash 
hailortcli --version
```
Checks that HailoRT was installed correctly by displaying the installed version. 

**Scanning for Devices**
```bash
hailortcli scan
```
Searches for connected Hailo devices and confirms that the system can detect the hardware.

**Identifying the Device Firmware**
```bash
hailortcli fw-control identify
```
Displays detailed information about the connected device, including firmware version and hardware details.

### Installing and configuring Docker

**Installing Docker** 
!!! info "**Docker Installation**"

    Install Docker by following the official documentation: [Docker Docs](https://docs.docker.com/engine/install/ubuntu/)

**Creating the Docker Group**

```bash
sudo groupadd docker 
```
Creates a docker group on the system. This group allows users to run Docker commands without needing ```sudo```.

**Adding User to Docker Group**
```bash
sudo usermod -aG docker $USER
```
Adds your current user to the docker group, granting permission to run Docker commands without elevated privileges.

**Applying Group Changes**
```bash
reboot
```
Restarts the systems so the group changes take effect.

**Testing Docker Installation**
```bash
docker run hello-world
sudo systemctl enable docker.service
```
Runs a test container to verify that Docker is working correctly. It also enables the Docker service so it starts automatically at boot.

**Enabling Container Runtime**
```bash
sudo systemctl enable containerd.service
```
Enables ```containerd```, the container runtime used by Docker, ensuring it also starts on boot.

**Disabling Docker Service (Optional)**
```bash
sudo systemctl disable docker.service
```
Disables the Docker service from starting automatically at boot. Use this only if you prefer to start Docker manually when need

### Building the container

**Cloning the project Repository**
```bash
git clone https://github.com/LichtenbergCode/drone-tello-leader-follower-vision-docker.git
```
Downloads the project repository containing the Docker setup and application code.

**Navigating to the Project Directory**
```bash
cd drone-tello-leader-follower-vision-docker
```
Moves into the project folder where the Docker configuration files are located.

**Allowing Docker to Access the Display (GUI Support)**
```bash
xhost +local:docker
```
Grants Docker containers permission to access the host's X server. This is required if the container runs application with a graphical interface.

!!! warning You need to run this command every time the container is used.

**Building the Docker Container**
```bash
docker compose build 
```
Builds the Docker image based on the ```docker-compose.yml``` configuration. This step installs all dependencies and prepares the environment.

**Running the container**
```bash
docker compose up -d hailo-ubuntu-pi-ros-jazzy hailo-ubuntu-pi-ros-jazzy
```
Starts the specified container in detached mode (```-d```). This launches the environment in the background so it can run independently of the terminal.

??? info ""**Changing the session type**""
    
    **Checking the Display Server Type**
    ```bash
    exho $XDG_SESSION_TYPE
    ```
    Displays the current session type (x11 or wayland).
    This is important because GUI application in Docker require X11 to work correctly.

    **Configuring Display Manager (if using Wayland)**
    ```bash
    sudo nano /etc/gdm3/custom.conf
    ```
    Opens the GDM configuration file. 
    If your system is using Wayland, you may need to disable it (by enabling X11) to ensure compatibility with Docker GUI applications.
    Change the line #WaylandEnable=false to WaylandEnable=false.
    
    **Applying Changes (if you change your session type)**
    ```bash
    reboot
    ```
    Restarts the system to apply any changes made to the display configuration.


**Accessing the Running Container**
```bash
docker compose exec hailo-ubuntu-pi-ros-jazzy
```
Opens an interactive shell inside the running container, allowing you to execute commands directly within it.

### Hailo Installation In The Container

**Installing Hailo Full Package**
```bash
sudo apt install hailo-all
```
Installs the complete Hailo software stack inside the container, including runtime libraries, tools, and dependencies required to work with the Hailo device.

**Verifying Hardware Detection**
```bash
hailortcli fw-control identify
```
Checks if the Hailo device is correctly detected inside the container.
If the installation is successful, it will display device information such as firmware version, architecture, and product details.

**Verifying TAPPAS Core Installation**
```bash
gst-inspect-1.0 hailotools
```
Confirms that the Hailo GStreamer plugins (TAPPAS core components) are properly installed.

If successful, it will list available Hailo elements used for building AI pipelines, such as detection, tracking, and post-processing.


**Installing Hailo Examples**
```bash
cd hailo-rpi5-examples 
./install.sh
```
Navigates to the Hailo examples directory and runs the installation script.

This sets up example pipelines and required resources for testing and development.

**Setting Up the Environment**
```bash
source setup_env.sh
```
Loads environment variables required for running Hailo applications, such as library paths and configuration settings.

To deactivate the virtual environment tap ```deactivate```


**Running a Test Pipeline**
```bash
python basic_pipelines/detection_simple.py
```
Executes a basic object detection example to verify that the full pipeline (hardware + software) is working correctly.


### Setting Up and Building ROS 2 Packages

**Sourcing ROS 2 Environment**
```bash
source /opt/ros/jazzy/setup.bash
```

**cvBridge installation**

open the src folder: 
```bash
cd vision_opencv/src
``` 
Compile the package: 
```bash
colcon build 
```
activate the environment you just compiled: 
```bash
source install/setup.bash
```
!!! quote "Quote"
    Check the repository to get more information: [Vision opencv](https://github.com/ros-perception/vision_opencv/tree/iron?tab=License-2-ov-file "vision-opencv")


**Building the ROS 2 interfaces for the project**

Compile the package:
```bash
cd drone-tello-leader-follower-interfaces
colcon build
```
Activate the environment you just compiled: 
```bash
source install/setup.bash
```

**Building the GUI and communication modules**

Open the repository: 
```bash
cd drone-tello-leader-follower
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

Ensure that your python virtual environment is activated: 
```bash
source venv/bin/activate
```
Install the necessary Python modules for the application: 
```bash
pip install -r requirements -v
```
Install numpy manually: 
```bash
pip install numpy==1.26.4
```
Compile the packages: 
```bash
colcon build
```
!!! warning "Use ```colcon build``` only within your workspace"

Activate the environment you just compiled: 
```bash
source install/setup.bash
```

**Building the Vision Package and modifying setup_env.sh**

Open hailo-rpi5-examples:
```bash
cd hailo-rpi5-examples 
```

Modify **setup_env.sh**: 

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


Build the vision package: 
```bash
cd drone-tello-leader-follower-vision
colcon build 
source install/setup.bash
```

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
### Module problems
!!! bug "**Issue: "ModuleNotFoundError: No module named 'matplotlib.tri.triangulation'"**"
    Uninstall matplotlib in your virtual environment: 
    ```bash
    pip uninstall matplotlib -y
    ``
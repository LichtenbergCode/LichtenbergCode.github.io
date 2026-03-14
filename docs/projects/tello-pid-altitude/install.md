icon:fontawesome/solid/gear

This section provides a guide to install and configure the environment required to run the DJI Tello altitude control system on Raspberry Pi 4. It covers dependency installation, ROS 2 Humble setup, and development environment configurations. 

<iframe width="100%" height="315" src="https://www.youtube.com/embed/njc3aHl7bSo?si=MYL-iXspfjrTWWJt" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## System Requirements

### Hardware Requirements
* Raspberry Pi 4 or 5 
* Storage: 32 GB microSD card (Class 10)
* DJI Tello Drone

###  Software Requirements
* Operating System: Ubuntu 22.04 LTS (64-bit, ARM) or Ubuntu 24.04 LTS (64-bit, ARM)
* ROS 2: Humble or Jazzy

## Environment Setup
### Update the system

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git python3-pip
```
### ROS 2 Installation
For Ubuntu 22.04 Install ROS 2 Humble: 
[ROS 2 Humble installation!](https://docs.ros.org/en/humble/Installation.html "ROS 2 Humble")

For Ubuntu 24.04 Install ROS 2 Jazzy: 
[ROS 2 Jazzy installation!](https://docs.ros.org/en/jazzy/Installation.html "ROS 2 Jazzy")

### Python Virtualenv Installation

To verify if **virtualenv** is ready installed, run: 
```bash
virtualenv --version
```

If **virtualenv** is not installed, run: 
```bash
sudo apt update && sudo apt install -y python3-virtualenv
```
## Project Cloning and Creating the virtual environment 

clone the repository and open the file: 
```bash
git clone https://github.com/LichtenbergCode/tello-pid-altitude-control-ros2.git
cd tello-pid-altitude-control-ros2
```

To create the virtual environment, run: 
```bash
virtualenv -p python3 ./venv
```

To use a virtual environment with ROS 2 you must tell colcon to ignore the directory that contains the files that manage the virtual environment: 
```bash
touch ./venv/COLCON_IGNORE
```

To ensure your ROS 2 distribution can access Python packages installed in your virtual environment, you need to explicitly add the virtual environment's Python path to ROS's Python system path. This bridges the gap between your isolated Python environment and the ROS 2 ecosystem.

Add the following lines to your virtual environment's activation script to automatically configure the Python path when the environment is activated:
```bash
export PYTHONPATH=${PYTHONPATH}:/path-to-your-env/lib/python3.10/site-packages
```
??? example "Example: Modify Virtual Environment Activation Script"
    
    !!! note "Your virtual environment's path or your python version will differ. Ensure you use the correct path for your venv name and Python version in the following commands"

    Navigate to your env-package path:
    ```bash
    cd ~/path/to/your/env-package
    ```
    Use the next command to get the absolute path of your current file: 
    ```bash
    pwd
    ```
    > ![env-package-path](img/install/venv-using-pwd.png "env-package-path")
    > environment package path

    navigate to your virtual environment activate file: 
    ```bash
    cd ~/path/to/your/activate_file
    ```
    open the activate file using a text editor:
    ```bash
    gedit activate
    ``` 
    > ![open-activate-file](img/install/venv-open-gedit.png "open-activate-file")
    > Open activate file

    Add the env-package path to your activate file: 
    ```bash
    export PYTHONPATH=${PYTHONPATH}:/path-to-your-env/lib/python3.10/site-packages
    ```
    > ![edit-activate-file](img/install/venv-edit-gedit.png "edit-activate-file")
    > Edit activate file


Activate your Python virtual environment: 
```bash
source venv/bin/activate
```

To exit your virtual virtual environment type: 
```bash
deactivate
```

Install the necessary Python modules for the application: 
```bash
pip install -r requirements -v
```

Compile the packages: 
```bash
colcon build
```
!!! warning "Use ```colcon build``` only within your drone_altitude_ws workspace" 
Activate the environment you just compiled: 
```bash
source install/setup.bash
```

Run the app: 
```bash
ros2 launch drone_altitude_bringup drone_altitude.launch.xml
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
    ```
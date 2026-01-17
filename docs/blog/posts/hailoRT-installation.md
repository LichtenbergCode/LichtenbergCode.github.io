---
date:
    created: 2026-01-16
    updated: 2026-01-17
readtime: 5
draft: true
categories: 
    - Hailo-8
    - Raspberry Pi
    - Linux
tags: 
    - technology
    - ARM64
    - IA
authors: 
    - alberto
---

# HailoRT Installation for ARM Architecture in Docker
Instructions for deploying HailoRT in a Docker container on ARM architecture to support accelerated AI inference on embedded platforms.

<!-- more -->

## Installation of Dependencies, Drivers, and the HailoRT SDK for Hailo-8 on Ubuntu Linux 24.04 LTS Systems Using a Raspberry Pi

This post section describes the procedure required to intall system dependencies, kernel headers, and the inference module drivers for the Hailo-8 accelerator in a Linux-base environment. 

**System Dependencies Installation**<br> 
Run the following command to install the base packages required for compilation, resource downloading, and systems configuration: 
```bash 
sudo apt update
sudo apt install -y build-essential cmake git unzip wget curl pkg-config ca-certificates gnupg software-properties-common
```

**Kernel Headers Installation**<br>
To compile kernel modules, it is necessary to install the headers corresponding to the currently running operating system version: 
```bash
sudo apt install -y linux-headers-$(uname -r)
```

**Installation of Hailo Drivers (hailort-drivers)**<br>

1. Clone the official drivers repository:
```bash
git clone https://github.com/hailo-ai/hailort-drivers.git
cd hailort-drivers
```

2. List available versions (tags) and select a stable one: 
```bash
git tag
git checkout v4.20.0
```

3. Compile and install the PCIe drivers: 
```
cd linux/pcie
sudo make install
sudo modprobe hailo_pci
```

4. Download and place the corresponding firmware
```
cd ../..
./download_firmware.sh
sudo mkdir -p /lib/firmware/hailo
sudo cp ./linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/
```

5. Reload udev rules so the system correctly recognized the device: 
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Ensure that paths and permissions are correctly configured. 
The firmware path is typically: 
```swift
/lib/firmware/hailo/hailo8_fw.bin
```
This file is created automatically during the process. 

**Installation Verification**
Reboot the system and run the following command to verify that the Hailo module has been correctly recognized: 
```bash
dmesg | grep hailo
```

**Installation and Verification of the HailoRT SDK**
1. Repository Cloning<br> 
Clone the official HailoRT repository and navigate to ist directory: 
```bash
git clone https://github.com/hailo-ai/hailort.git
cd hailort
```
2. Stable Version Selection<br>
To ensure environment stability, it is recommended to version **4.20.0**:
```bash
git checkout v4.20.0
```
3. Compilation and Installation<br>
Th project is build using CMake. The following command compile the SDK in **Release** mode and install in system-wide:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DHAILO_BUILD_EXAMPLES=ON
sudo cmake --build build --config Release --target install
```
4. Installation Verification<br>
After installation, verify that ```hailortcli```, the SDK command-line tool, is properly installed and accessible:
```bash
hailortcli --version
```
5. Device Identification<br>
To confirm that the system correctly recognizes the Hailo-8 module, execute: 
```bash
hailortcli fw_control identify
```
## Creation of the Docker Container for the Hailo Environment on Raspberry Pi

**Cloning the base Repository** <br>
The official Canonical repository was used as the base, as it provides the necessary structure to initialize a container compatible with the Raspberry Pi AI Kit under Ubuntu 24.04:

```bash
git clone https://github.com/canonical/pi-ai-kit-ubuntu.git
cd pi-ai-kit-ubuntu
```

**Graphical Access Configuration** <br>
To enable image visualization tools or the execution of GUI-based applications, it is necessary to authorize the container's access to the host X server using the following command: 
```bash
xhost +local:docker
```
This command allows Docker containers running locally to interact with graphical display of the host system. 

**Container Image Build** <br>
The Docker image is built using the configuration files provided in the cloned repository by executing: 
```bash
docker compose build
```
This process compiles all components defined in the ```docker-compose.yml ``` file, including the installation of the base operating system, development libraries, and the Hailo module SDK. 

**Running the Container in Datached Mode** <br>
Once the image has been successfully built, the container is launched in detached mode using: 
```bash
docker compose up -d hailo-ubuntu-pi
```
This command starts the ```hailo-ubuntu-pi``` container in the background (````-d```), leaving it active and ready for immediate use. 

**Accessing the Container Environment**<br>
To interactively access the deployed container environment, the following command is used: 
```bash
docker compose attach hailo-ubuntu-pi
```
This allows direct interaction with the system running inside the container, where all required tools for development and inference using the **Hailo-8 module** are available
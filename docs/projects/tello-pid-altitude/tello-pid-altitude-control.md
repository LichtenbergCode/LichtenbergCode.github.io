icon:octicons/graph-16

# DJI Tello Altitude PID Controller
This project provides a real-time altitude control system for the DJI Tello drone, running on a **Raspberry Pi 4**. It implements a Proportional-Integral-Derivative (PID) controller to maintain stable flight at a desired height. The system leverages **DJITellopy** for reliable low-level communication with the drone, **ROS 2** for robust, node-based inter-process communication on the Pi, and features a graphical user interface (GUI) built with **Customtkinter** for intuitive control and real-time visualization of key flight parameters using **Matplotlib**.

## Introduction 
This project is a complete embedded control system designed to demonstrate closed-loop altitude control for the DJI Tello micro-drone. The core motivation is to create a practical, educational platform that bridges the gap between theoretical control systems and real-world robotics deployment, showcasing how a Raspberry Pi 4 can act as an onboard brain for autonomous drone operations. It addresses the need for a customizable and analyzable control system by replacing the drone's built-in altitude hold with a custom PID controller, providing a foundational block for more complex autonomous behaviors. By integrating ROS 2 for modularity, a modern GUI for interaction, and real-time data visualization for analysis. The system is currently in a stable prototype stage, with all core functionalities—including flight control, visualization, and user interface—fully operational on the Raspberry Pi platform.

## Project Goals and Objectives

### Primary Goals:
* Develop a reliable and tunable PID controller on a Raspberry Pi 4 to accurately maintain the DJI Tello's altitude.

* Create a robust, self-contained system where all computation (ROS 2 nodes, GUI, PID logic) runs directly on the Raspberry Pi, communicating with the drone via WiFi.

* Provide a real-time visualization of the control loop's performance on the Pi's display, including error, setpoint, actual altitude, and system output.

### Secondary Objectives:
* Create an efficient GUI using Customtkinter that performs well on the Raspberry Pi's hardware, allowing for on-the-fly PID tuning and monitoring during flight.

* Implement battery monitoring and safety shutdown procedures to protect hardware during field operations. 

### Long-term vision

* Use this Raspberry Pi-based controller as a template for a swarm of drones, where each Pi acts as an autonomous node in a distributed system.

* Integrate additional sensors (e.g., a camera for OpenCV-based navigation) directly with the Pi to create a fully autonomous drone platform.

## Key Features
* **Raspberry Pi 4 Deployment**: The entire control system—including the ROS 2 network, PID controller, and GUI—is designed to run on a Raspberry Pi 4, demonstrating embedded control system design.

* **Custom PID Controller**: A software-based PID implementation running on the Pi for precise, onboard altitude regulation. 

* **DJITellopy Integration**: Utilizes a distributed node system on the Pi itself for modular and decoupled communication between the GUI, PID logic, and hardware interface.

* **Real-time Data Visualization**:  Integrated Matplotlib plots within the GUI to provide immediate visual feedback on the control system's performance, aiding in PID tuning and system analysis directly on the embedded device.

* **Modern Graphical User Interface**: A user-friendly interface built with Customtkinter, optimazed for use on the Raspberry Pi's display.

## System Summary


The system is built primarily with Python on a **Raspberry Pi 4** running a Linux-based OS (Ubuntu 22.04 LTS). It utilizes key software frameworks: **ROS 2** (Humble) for inter-process communication on the Pi, **DJITellopy** as the core drone SDK, **Customtkinter** for the graphical interface, and **Matplotlib** for dynamic plotting. At a high level, the Raspberry Pi hosts all components: it connects to the Tello's WiFi network, and then runs the GUI, PID controller, and DJITellopy interface as separate ROS 2 nodes. The PID node subscribes to the drone's altitude data (provided by a DJITellopy wrapper node), calculates the control output, and publishes commands, which the DJITellopy node sends to the drone. All data is visualized in real-time on the GUI displayed on the Pi.

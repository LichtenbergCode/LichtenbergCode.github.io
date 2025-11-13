icon:material/quadcopter

<img src="../img/overview/tello.png" alt="tello-drone" align="right" width=40%>

A ground control station for DJI Tello drones featuring dual operational modes (Master/Slave) with real-time PID gain adjustment and comprehensive telemetry monitoring. Built on a modular ROS 2 architecture with dynamic control scheme switching capabilities. 

## Introduction 

This project provides a dual-mode control interface for DJI Tello drones, enabling both autonomous follower operations and manual piloting through an intuitive graphical interface. The primary motivation is to create a versatile control platform that bridges manual operation and autonomous swarm behaviors, addressing the gap between basic remote controls and complex autonomous systems. It solves the challenge of dynamic control scheme switching while maintaining real-time parameter adjustment and system monitoring.

## Project Goals and Objectives 

### Primary Goals: 
* Develop a dual-mode control system (Master/Slave) with seamless switching capabilities. 

* Implement four independent PID controllers for precise multi-axis trajectory tracking. 

* Create a modular GUI architecture for real-time parameter adjustment and system monitoring. 

### Secondary Objectives: 
* Design interactive visualization tools for controller response analysis and gain tuning. 

* Establish robust ROS 2 communication for reliable multi-drone coordination

* Implement safety protocols for mode transitions and emergency operations. 

## Key Features
* **Dual Operational Modes**: Master (manual) and Slave (autonomous follower) with dynamic switching. 

* **Multi-axis PID Control**: Four independent controllers fot precise movement coordination. 

* **Real-time Gain Adjustment**: Dynamic PID parameter tining during operation

* **Interactive Visualization**: Live controller response monitoring and performance analysis. 

* **Modular GUI Architecture**: Segmented interface for commands, information, and visualization. 

* **Telemetry**: Real-time monitoring of critical flight parameters and system status. 

* **ROS 2 Integration**: Distributed node system ensuring reliable communication and data synchronization. 

## System Summary
The system leverages Python with CustomTkinter, ROS 2 Iron for high-performance inter-process communication, and integrates the Hailo AI accelerator module through the hailo-apps-infra-environment for advanced onboard on Raspberry Pi OS running on a Raspberry Pi 5. The architecture maintains a segmented GUI design with dedicated functional areas - Panels buttons segmentation (left), sensors data (right), power/ communication management (top), and video/graphs visualization (center)- while utilizing ROS 2 Iron's enhanced performance for real-time data exchange between control nodes. The Hailo AI integration enables vision tasks directly on the raspberry Pi 5, providing real-time object detection and tracking capabilities that enhance both Master and Slave operational modes. 
 
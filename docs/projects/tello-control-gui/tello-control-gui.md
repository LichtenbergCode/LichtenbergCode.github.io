---
icon: material/television-guide
---

A comprehensive ground control station for manual flight operations of the DJI Tello drone, featuring intuitive dual-joystick style controls, real-time telemetry monitoring, and live video streaming.

## Introduction 
This project delivers a user-friendly basic control interface for the DJI Tello drone, enabling precise manual flight operations through an intuitive graphical interface. 

## Project Goals and Objectives

### Primary Goals: 
* Develop an intuitive dual-joystick control interface for precise manual drone operation.

* Implement real-time video streaming and telemetry display for situational awareness. 

* Create a robust ROS 2 based communication system ensuring reliable drone connectivity. 

### Secondary Objectives: 
* Design a modern, responsive GUI using Ttkbootstrap for enhanced user experience.

* Integrate comprehensive safety features including emergency stop and connection monitoring.

* Provide modular code architecture for easy extension and customization

## Key Features

* **Dual Control Interface**: Separate button sets for Pitch/Roll and Yaw/Throttle providing joystick-like precision.

* **Real-time Video Streaming**: Live camera feed.

* **Comprehensive Telemetry Dashboard**: Battery level, temperature, and flight status indicators.

* **One-Touch Operations**: Instant connect, takeoff, and landing with safety verification. 

* **Modern GUI Framework**: Ttkbootstrap-based interface with theme support and responsive design. 

* **ROS 2 Integration**: Distributed node architecture for reliable communication and system modularity.

* **Raspberry Pi Optimized**: Lightweight implementation designed for embedded system performance.

## System Summary

The System leverages **Python** with **Ttkbootstrap** for the graphical interface, **ROS 2 Humble** for inter-process communication, and DJITellopy for low-level drone control, all running on **Ubuntu 22.04** on a **Raspberry Pi 4**. The architecture consists of two main components: a GUI node handling user input and display and a communication node managing drone connectivity. These components interact through ROS 2 topics and services, ensuring real-time responsiveness while maintaining system stability and modularity. 

## Repository Contents

project/
├── docs/
│   ├── Overview.md
│   ├── Installation.md
│   ├── UserGuide.md
│   ├── Features.md
│   ├── Functionalities.md
│   └── Architecture.md
├── src/
├── config/
└── docker/
 

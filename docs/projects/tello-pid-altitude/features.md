icon:material/newspaper-variant-multiple


## 1. Overview

This section provides a detailed description of the main **features** implemented in the project.  
Each feature contributes to the system’s functionality, stability, and user experience.

> 💡 **Purpose:**  
> The goal of this document is to help users understand what the system can do, how each feature works, and what makes it unique or technically interesting.

---

## 2. Core Features

### 🧠 2.1 Intelligent Control System
The core of the project is an **intelligent PID-based control system** that automatically regulates the drone’s flight parameters, ensuring smooth and precise altitude management.

**Highlights:**
- Real-time sensor feedback (IMU, barometer, camera).
- Adaptive PID tuning based on flight conditions.
- Fail-safe mechanisms for stability recovery.
- Manual override through GUI or keyboard input.

> Example log output:
> ```
> [PID] Target: 1.20 m | Measured: 1.18 m | Output: +0.05 | Status: Stable
> ```

[Insert diagram or illustration of control loop here]

---

### 📡 2.2 ROS 2 Integration
This project is fully compatible with **ROS 2 (Robot Operating System)**, allowing distributed communication between nodes and modular scalability.

**Key Features:**
- ROS 2 topics for telemetry, images, and control commands.  
- Parameterized nodes for configurable behavior.  
- Support for DDS-based networking (even across multiple devices).  
- Compatible with `ros2 topic`, `rqt_graph`, and `rviz2`.

> 💡 Example:
> ```bash
> ros2 topic list
> /tello/image_raw  
> /tello/cmd_vel  
> /tello/altitude  
> ```

---

### 🖥️ 2.3 Graphical User Interface (GUI)
An interactive, minimalistic GUI built with **CustomTkinter** enables real-time control and visualization.

**Includes:**
- Dynamic theme switching (light/dark mode).  
- Live camera streaming and telemetry view.  
- Configurable sliders for PID parameters.  
- Buttons for connection, calibration, and emergency stop.

[Insert screenshot of GUI interface here]

> The GUI automatically adapts to the system theme and screen size.

---

### ⚙️ 2.4 Configurable Parameters
The entire control logic can be customized via a single configuration file (`config.yaml`), enabling flexible deployment.

| Parameter | Type | Description | Example |
|------------|------|-------------|----------|
| `control_mode` | string | Choose control algorithm (`PID`, `LQR`, etc.) | PID |
| `kp`, `ki`, `kd` | float | PID coefficients | 1.0 / 0.01 / 0.7 |
| `max_altitude` | float | Limit to prevent unsafe operation | 2.5 |
| `data_logging` | bool | Enable CSV data recording | true |

> ⚙️ **Advanced Tip:** You can load a different configuration dynamically at runtime:
> ```bash
> python main.py --config configs/experiment.yaml
> ```

---

### 🧩 2.5 Modular Architecture
The system is organized in a **modular architecture** to simplify maintenance and scalability.

**Modules include:**
1. **Core:** control algorithms and flight logic  
2. **GUI:** user interaction layer  
3. **Networking:** ROS 2 and socket communication  
4. **Logging:** data storage and visualization  
5. **Utils:** helper functions, math, and filters  

> This structure allows for independent testing and easy integration with other robotics frameworks.

[Insert architecture block diagram here]

---

### 🧪 2.6 Simulation and Testing
The software includes support for **virtual testing environments** and debugging tools.

**Available options:**
- Simulated PID control (without real hardware).  
- Real-time plotting of control variables.  
- Logging of altitude, roll, and yaw responses.  
- Test scripts for validating stability and latency.

> Example command:
> ```bash
> python simulation.py --mode pid_test
> ```

---

### 🌈 2.7 Theme-Aware Design
All interface components adapt dynamically to the current theme (light or dark mode).

**Features:**
- Uses Material for MkDocs `data-md-color-scheme` for automatic color switching.  
- Changes icons, images, and background colors seamlessly.  
- Custom CSS variables (`--my-*`) allow further visual customization.

> 💡 **Example CSS snippet:**
> ```css
> [data-md-color-scheme="slate"] .custom-hero {
>   background-image: url("images/dark-bg.png");
> }
> [data-md-color-scheme="default"] .custom-hero {
>   background-image: url("images/light-bg.png");
> }
> ```

---

## 3. Optional Features

### 🔌 3.1 Docker Integration
Easily deployable using **Docker containers** for reproducibility and isolated environments.

- Based on Ubuntu or Debian image.
- Includes Python, ROS 2, and Hailo SDK support.
- Pre-configured volume mounting and networking.

**Example command:**
```bash
docker build -t tello-control .
docker run -it --network host tello-control
```

Limitations
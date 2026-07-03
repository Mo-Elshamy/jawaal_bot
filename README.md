# Jawbot Autonomous Mobile Robot (AMR)

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-brightgreen.svg)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

## 1. Overview
Jawbot is a custom-built Autonomous Mobile Robot (AMR) designed around a differential drive kinematics system. It aims to bridge low-level microcontrollers (Raspberry Pi Pico) with high-level ROS 2 navigation stacks, providing a robust, extensible platform for research, education, and development of mobile robotics applications.

## 2. Key Features
- **Diff-Drive Kinematics:** Fully supported by the ROS 2 `diff_drive_controller` and `joint_state_broadcaster`.
- **Custom Hardware Interface:** Uses a custom `ros2_control` `SystemInterface` to communicate seamlessly with the microcontroller.
- **Real-Time Closed-Loop Control:** PID velocity tracking executed deterministically on the Raspberry Pi Pico at 50Hz.
- **Simulation Ready:** Full integration with Gazebo (Harmonic) for physics simulation and testing prior to hardware deployment.
- **Modular Architecture:** Packages are neatly decoupled, isolating hardware drivers from kinematics and visualization.

## 3. System Architecture Stack
Jawbot operates on a decoupled, full-stack architecture:

- **Hardware Layer:** Raspberry Pi Pico (Main MCU), L298N Motor Drivers, and Quadrature Encoders. Handles real-time motor control and interrupt-driven sensor tracking.
- **Software Layer:** Runs on Ubuntu 24.04 with ROS 2 Jazzy. Manages high-level velocity commands (`cmd_vel`), TF trees, and state publishing.
- **Control Bridge:** A robust, non-blocking ASCII Serial Protocol over `/dev/ttyACM0` connecting the Linux environment to the hardware.

*For a detailed deep-dive into the control architecture, firmware design, and serial protocol, please see the [Control Architecture Documentation](jawbot_hardware/Control_Arc.md).*

## 4. Repository Structure
The project is divided into several focused ROS 2 packages:

- **`jawbot_description`**: Contains the URDF, Xacro macros, and 3D meshes defining the robot's physical properties.
- **`jawbot_hardware`**: The core C++ `ros2_control` hardware plugin and Linux serial communication drivers.
- **`jawbot_gazebo`**: Launch files and world definitions for Gazebo simulation.
- **`jawbot_bringup`**: Orchestrates the launch sequences, starting the robot state publisher, controller manager, and spawning controllers.
- **`jawbot_navigation`**: Configuration files for the ROS 2 Nav2 stack, including costmaps and SLAM parameters.

## 5. Prerequisites & Dependencies
- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Build Tool:** `colcon`
- **Dependencies:**
  - `ros2_control` & `ros2_controllers`
  - `robot_state_publisher`
  - `xacro`
  - `gazebo_ros_pkgs`
  - `nav2`

### Hardware Requirements
- Host computer / SBC running Linux
- Raspberry Pi Pico (flashed with `jawbot_pico` firmware)
- USB Serial connection to the Pico (e.g., `/dev/ttyACM0`)

## 6. Simulation Deployment Guide

This section outlines how to set up, build, and run the robot in a simulated Gazebo environment on your local development machine.

### Step 1: Set Up Workspace & Clone Packages
```bash
mkdir -p ~/test_ws/src
cd ~/test_ws/src
# Clone the jawbot repository here
```

### Step 2: Install Dependencies
```bash
cd ~/test_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### Step 3: Compile the Workspace
```bash
colcon build --symlink-install
```

### Step 4: Sourcing and Running the Simulation
```bash
source install/setup.bash
ros2 launch jawbot_gazebo gazebo.launch.py
```

### Teleoperation
To drive the robot in the simulation, open a new terminal and run:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 7. Real Hardware Deployment Guide (Wireless / Pi Mode)

This section outlines how to deploy, compile, and run the production robotics stack directly on the robot's onboard Single Board Computer (SBC) over a wireless network link.

### Prerequisites
* The SBC must be powered, connected to the same local Wi-Fi subnet as your development laptop, and have SSH enabled.
* The low-level microcontroller (e.g., Pi Pico, ESP32) must be physically attached to the SBC via a USB data cable.
* ROS 2 must already be natively installed on the SBC operating system.

---

### Step 1: Establish Remote Connection (SSH)
Open a terminal window on your laptop and log into the robot’s wireless terminal interface. Replace `<username>` and `<hostname>` with your robot's configured credentials:

```bash
ssh <username>@<hostname>.local
# (Alternatively, connect using the robot's direct IP address: ssh <username>@<ip_address>)
```

### Step 2: Set Up Workspace & Clone Packages
Create a dedicated developer workspace, navigate to its source folder, and pull down your robot controller repositories:

```bash
# Create the workspace directory structure
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone your robot code repositories
git clone <your_repository_git_url> .
```

### Step 3: Configure Hardware Access Permissions
Linux secures hardware serial lines by default. Grant your user account permanent permission to read and write data over the physical USB serial port interfaces:

```bash
# Add the active user to the dialout hardware communication group
sudo usermod -aG dialout $USER

# Force-apply the group updates without requiring a system reboot
newgrp dialout
# (Verify your microcontroller is registered by running ls /dev/ttyACM* or ls /dev/ttyUSB*)
```

### Step 4: Resolve Dependencies Automatically
Scan the cloned repository package definitions (`package.xml`) and pull down any missing drivers, middleware, or operational plugins required by the hardware control loops:

```bash
cd ~/robot_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO
```

### Step 5: Compile the Workspace
Compile the package nodes and hardware interface plugins directly on the single-board computer architecture. We use symbolic link maps to allow modification of launch and config files without re-building:

```bash
cd ~/robot_ws
colcon build --symlink-install
```

### Step 6: Sourcing and Running the Robot Bringup
Load the freshly compiled package targets into your environment variables and execute the master real-hardware orchestrator launch file:

```bash
# Source the workspace setup profile
source install/setup.bash

# Run the master bringup launch file
ros2 launch <your_bringup_package_name> <your_real_bringup_launch_file>.launch.py
```

### Teleoperation
Once the robot is running, you can teleoperate it from a new SSH terminal or a connected laptop (provided `ROS_DOMAIN_ID` is set correctly):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 8. License & Acknowledgments
- **License:** MIT License
- Designed and built as a modular AMR platform.
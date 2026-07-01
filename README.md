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

## 6. Installation & Build Instructions

1. **Source ROS 2:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **Clone the Repository:**
   ```bash
   mkdir -p ~/test_ws/src
   cd ~/test_ws/src
   # Clone the jawbot repository here
   ```

3. **Install Dependencies:**
   ```bash
   cd ~/test_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the Workspace:**
   ```bash
   colcon build --symlink-install
   ```

5. **Source the Workspace:**
   ```bash
   source install/setup.bash
   ```

## 7. Usage & Quick Start

### Simulation Mode (Gazebo)
To test the robot in a simulated environment without physical hardware:
```bash
ros2 launch jawbot_gazebo gazebo.launch.py
```

### Hardware Mode (Real Robot)
*Ensure your Pi Pico is plugged in and recognized as `/dev/ttyACM0` (or update the `port` argument).*
```bash
ros2 launch jawbot_bringup real_bringup.launch.py port:=/dev/ttyACM0
```

### Teleoperation
Once the robot (simulated or real) is running, open a new terminal and run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the keyboard to send velocity commands to the `diff_drive_controller`.

## 8. License & Acknowledgments
- **License:** MIT License
- Designed and built as a modular AMR platform.
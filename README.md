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
- **RPLiDAR A1 Integration:** Dedicated ROS 2 node setup for real-time 2D laser scan publishing (`/scan`).
- **2D SLAM & Mapping:** Parameterized `slam_toolbox` configuration for online asynchronous mapping and map saving.
- **Simulation Ready:** Full integration with Gazebo (Harmonic) for physics simulation and testing prior to hardware deployment.
- **Modular Architecture:** Packages are neatly decoupled, isolating hardware drivers from kinematics, localization, and navigation.

## 3. System Architecture Stack
Jawbot operates on a decoupled, full-stack architecture:

- **Hardware Layer:** Raspberry Pi Pico (Main MCU), L298N Motor Drivers, Quadrature Encoders, and RPLiDAR A1 M8. Handles real-time motor control, interrupt-driven sensor tracking, and 2D laser range scanning.
- **Software Layer:** Runs on Ubuntu 24.04 with ROS 2 Jazzy. Manages high-level velocity commands (`cmd_vel`), TF trees, state publishing, EKF sensor fusion (50Hz), and `slam_toolbox` mapping.
- **Control Bridge:** A robust, non-blocking ASCII Serial Protocol over `/dev/ttyACM0` connecting the Linux environment to the hardware.

*For a detailed deep-dive into the control architecture, firmware design, and serial protocol, please see the [Control Architecture Documentation](jawbot_hardware/Control_Arc.md).*
*For step-by-step procedures on calibrating odometry (wheel radius, track width, encoder ticks) and IMU offsets (GY-87), please see the [Sensor & Odometry Calibration Guide](SENSOR_CALIBRATION.md).*

## 4. Repository Structure
The project is divided into several focused ROS 2 packages:

- **`jawbot_description`**: Contains the URDF, Xacro macros, and 3D meshes defining the robot's physical properties.
- **`jawbot_hardware`**: The core C++ `ros2_control` hardware plugin, Linux serial communication drivers, and RPLiDAR launch configuration.
- **`jawbot_gazebo`**: Launch files and world definitions for Gazebo simulation.
- **`jawbot_bringup`**: Orchestrates the launch sequences, starting the robot state publisher, controller manager, and spawning controllers.
- **`jawbot_localization`**: Extended Kalman Filter (EKF) state estimation fusing raw IMU data and wheel odometry at 50Hz.
- **`jawbot_navigation`**: Configuration files for ROS 2 Nav2 and 2D SLAM mapping (`slam_toolbox` params, launch scripts, and saved maps).

## 5. Prerequisites & Dependencies
- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Build Tool:** `colcon`
- **Dependencies:**
  - `ros2_control` & `ros2_controllers`
  - `robot_state_publisher`
  - `xacro`
  - `gazebo_ros_pkgs`
  - `robot_localization`
  - `rplidar_ros`
  - `slam_toolbox`
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
ros2 launch jawbot_bringup real_bringup.launch.py

```

### Teleoperation

Once the robot is running, you can teleoperate it from a new SSH terminal or a connected laptop (provided `ROS_DOMAIN_ID` is set correctly):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

### Step 7: Dynamic PID Controller Tuning

To achieve smooth and accurate movements, Jawbot utilizes an embedded C++ node to dynamically tune the Pi Pico's motor PID loops mid-flight.

**1. Launch the Visualization Tools:**
On your local development laptop, run the dynamic parameter GUI and the real-time plotter tool:

```bash
# Terminal 1: Open the parameter sliders
ros2 run rqt_reconfigure rqt_reconfigure

# Terminal 2: Open the velocity plotter pre-loaded with left/right wheel velocity topics
ros2 run rqt_plot rqt_plot /joint_states/velocity[0] /joint_states/velocity[1]

```

**2. Tune the Robot Mid-Flight:**

* Safely place the robot on an elevated block so the wheels can spin freely in the air.
* Use your teleop terminal to send a constant velocity command (e.g., driving straight forward).
* In `rqt_reconfigure`, select the `jawbot_pid_tuner` node. Use the sliders to dynamically adjust `motor_kp`, `motor_ki`, and `motor_kd`.
* Watch `rqt_plot` (monitoring the `/joint_states` velocity data) to ensure the actual wheel speed smoothly converges on your target speed without violent oscillations.

> ⚠️ **Common Pitfall — Plotting Array Fields in `rqt_plot`:**  
> Unlike RViz, `rqt_plot` cannot plot entire ROS message structs; it requires single numerical variables (floats/integers). Typing `/joint_states` into the Topic bar will show nothing because it is a complex message containing arrays.  
>  
> **The Fix:** In the `rqt_plot` text bar, type the exact numeric field index and click the green **`+`** button:  
> - **Left Wheel Velocity:** `/joint_states/velocity[0]`  
> - **Right Wheel Velocity:** `/joint_states/velocity[1]`  
> - **Alternative (Odom Linear Speed):** `/jawbot_base_controller/odom/twist/twist/linear/x`

* Finally, place the robot on the floor and slightly increase `motor_ki` to help the robot push through physical ground friction.

**3. Hardcode and Save the Final Values:**
The `rqt_reconfigure` GUI only overwrites the PID values temporarily in active memory. Once you have found your ideal "Magic Numbers" from the floor test, you must permanently save them to keep the system synchronized on boot:

* **The C++ ROS 2 Interface (The GUI Default):** Open your hardware implementation  file (e.g., `jawbot_hardware_interface.cpp`). Update the starting state variables (`current_kp_`, `current_ki_`, `current_kd_`) so the GUI sliders correctly reflect the Pico's default state the next time you open them.

### Step 8: Sensor & Odometry Calibration

Before deploying high-level navigation stacks (Nav2 / SLAM), ensure your robot's physical kinematics and IMU sensors are properly calibrated to eliminate drift:
* Follow the **[Sensor & Odometry Calibration Guide](SENSOR_CALIBRATION.md)** for step-by-step instructions on calibrating encoder ticks per revolution, effective wheel radius, track width, and GY-87 accelerometer/gyroscope zero-offsets.

---

### Step 9: 2D SLAM Mapping & Map Saving

Jawbot uses `slam_toolbox` in online asynchronous mapping mode coupled with RPLiDAR scan data for high-quality 2D occupancy grid mapping.

#### 1. Launch Hardware Bringup & RPLiDAR
On the robot (or in bringup launch sequence), ensure the real hardware driver and RPLiDAR node are running:
```bash
# Launch master hardware bringup (which launches real hardware interface + RPLiDAR)
ros2 launch jawbot_bringup real_bringup.launch.py
```

#### 2. Launch SLAM Toolbox
Start the SLAM node with the custom `slam_toolbox_params.yaml` configuration:
```bash
ros2 launch jawbot_navigation slam.launch.py
```

#### 3. Teleoperate and Build Map
Drive the robot around the room using keyboard teleop to cover all areas:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 4. Save the Generated Map
Once the occupancy grid map is complete, save the map files (`.pgm` image and `.yaml` metadata) to `jawbot_navigation/maps/`:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/jawaal_bot/jawbot_navigation/maps/my_room_1
```

---

## 8. License & Acknowledgments
- **License:** MIT License
- Designed and built as a modular AMR platform.

Notes:
1- the ticks per revolution of the motor is 988 and need to be configured in both jawbot_ros2_control.urdf.xacro and jawbot_pico.ino files
2- the pid params must be set in the jawbot_hardware_cpp and jaw.ino file

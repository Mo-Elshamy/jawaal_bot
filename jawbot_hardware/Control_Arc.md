# Jawbot Autonomous Mobile Robot: Control Architecture Documentation

This document outlines the full-stack control architecture for the Jawbot AMR. The system uses a modular, decoupled approach that separates high-level ROS 2 kinematic planning from low-level deterministic motor control, bridged by a robust serial communication layer.

---

## Part 1: Microcontroller Firmware (Raspberry Pi Pico)

### 1.1 Methodology
The Pico acts as a real-time execution slave. The architecture uses Object-Oriented C++ principles, abandoning blocking functions (like `delay()`) in favor of hardware-timed interrupts. The methodology relies on a **50Hz Hardware Timer** to execute PID velocity math, paired with **Hardware Interrupt Service Routines (ISRs)** to capture quadrature encoder pulses instantly. It features a **Dual-Stage Friction Compensation and Feedforward System**:
1. **Static Breakaway Kick (`STATIC_BREAKAWAY = 38.0%`):** Provides an initial power boost when starting from low speeds ($<0.25$ rad/s) to overcome static mechanical friction (stiction).
2. **Kinetic Assist (`KINETIC_ASSIST = 33.0%`):** Tapers down to a steady rolling offset once the wheels are in motion.
3. **Feedforward Voltage Scaling (`KV_FEEDFORWARD = 3.3`):** Scales PWM effort proportionally with target velocity for rapid transient response.
4. **Hard Stop & Integral Reset:** Instantly resets PID integral memory (`pid_.reset()`) and zeros motor output when target velocity is zero ($<0.01$ rad/s) to eliminate post-stop wheel creep/oscillations.

### 1.2 Files and Directory
| File Path | Description |
| :--- | :--- |
| `jawbot_pico.ino` | Main dispatcher, hardware timer setup, and communication heartbeat. |
| `L298N.h/.cpp` | Actuator physical layer (PWM and Direction logic). |
| `WheelEncoder.h/.cpp` | Sensor physical layer (Interrupt-driven quadrature tracking). |
| `gy87_driver.h/.cpp` | Sensor physical layer (I2C communication with GY-87 IMU module). |
| `pid.h/.cpp` | Mathematical engine for closed-loop velocity tracking. |
| `RobotJoint.h/.cpp` | Abstraction layer wrapping Motor, Encoder, PID, dual-stage friction offset, and feedforward logic. |

### 1.3 Classes and Functions
| Class | Function | Description |
| :--- | :--- | :--- |
| **`L298N`** | `init()`, `setSpeed()` | Configures GPIOs and maps velocity to PWM signals. |
| **`WheelEncoder`**| `init()`, `handleInterrupt()`, `getTicks()` | Manages quadrature logic via ISRs and safe tick retrieval. |
| **`GY87`** | `init()`, `update()` | Initializes I2C bypass for MPU6050 and HMC5883L, and populates IMU struct with scaled accelerometer, gyroscope, and magnetometer data. |
| **`PIDController`**| `compute()`, `setGains()` | Executes velocity tracking math and supports live gain tuning. |
| **`RobotJoint`** | `executeControlLoop()` | Cascades logic: Encoder $\to$ Velocity Calc $\to$ Hard Stop Check $\to$ PID $\to$ Feedforward & Dual-Stage Friction Compensation $\to$ Saturation Limiter $\to$ Motor PWM. |

---

## Part 2: ROS 2 Control (Hardware Interface Plugin)

### 2.1 Methodology
This layer runs on the Linux SBC using the `ros2_control` framework. It acts as a translator between ROS 2 joint state/command interfaces and the low-level serial protocol. It runs as a `SystemInterface` plugin loaded by the `controller_manager`, ensuring the robot's hardware state is synchronized with the URDF model. This includes exposing wheel joint states as well as **IMU Sensor State Interfaces** (accelerometer, gyroscope, and magnetometer) for the `imu_sensor_broadcaster`. Additionally, it hosts an embedded parameter node allowing for **Dynamic PID Tuning** mid-flight via `rqt_reconfigure`.

### 2.2 Files and Directory
| File Path | Description |
| :--- | :--- |
| `include/jawbot_hardware/interface.hpp` | Interface declarations and lifecycle hooks. |
| `src/jawbot_hardware_interface.cpp` | Implementation of `read()`/`write()` loops and serial translation. |
| `jawbot_hardware.xml` | Plugin registration file for `pluginlib`. |

### 2.3 Classes and Functions
| Class | Function | Description |
| :--- | :--- | :--- |
| **`JawbotHardwareInterface`** | `on_init()` | Parses hardware URDF parameters and initializes state arrays. |
| | `read()` | Ingests serial telemetry and calculates joint velocity via timestamps. |
| | `write()` | Formats target velocities into string commands for the microcontroller. |
| | `on_activate()` | Lifecycle hook: Initializes port and performs safety zeroing of motors. |
| | (Embedded Node) | A background thread (`tune_thread_`) listens for `rqt_reconfigure` parameter updates and syncs them to the Pico. |

---

## Part 3: Serial Communication Layer

### 3.1 Methodology
A **Non-Blocking ASCII Serial Protocol** bridge. This layer ensures robust cross-process communication between Linux and the Pico. It features a watchdog mechanism that monitors command latency; if communication stops (e.g., node crash), the firmware autonomously brakes the robot for safety. The protocol multiplexes high-frequency wheel telemetry with periodic IMU (GY-87) sensor data packets.

### 3.2 Files and Directory
| File Path | Description |
| :--- | :--- |
| `include/jawbot_hardware/serial_comms.hpp` | POSIX Linux serial driver header. |
| `src/serial_comms.cpp` | Raw termios setup and non-blocking read/write implementation. |
| `SerialProtocol.h/.cpp` (Pico) | Parsing logic, telemetry formatting, and watchdog timers. |

### 3.3 Classes and Functions
| Class | Function | Description |
| :--- | :--- | :--- |
| **`SerialComms` (Linux)** | `connect()`, `read_msg()`, `write_msg()` | Manages Linux file descriptors and non-blocking serial I/O. |
| **`SerialProtocol` (Pico)** | `processIncoming()`, `sendTelemetry()` | Parses ASCII commands (`m`, `p`) and sends heartbeat telemetry (both standard wheel-only and overloaded IMU data versions). |
| | `getLastCommandTime()` | Used by the main firmware to monitor watchdog status and blink status LEDs. |

---

## Part 4: Localization & Nav2 Autonomous Navigation

### 4.1 AMCL & Map-Based Pose Estimation
- **Map Server:** Loads static 2D occupancy grid maps (`.yaml`/`.pgm`) generated via `slam_toolbox`.
- **AMCL (Adaptive Monte Carlo Localization):** Uses likelihood field range measurement modeling with differential motion model updates to estimate robot pose (`map` $\to$ `odom` transform).

### 4.2 Nav2 Architecture Pipeline
- **BT Navigator (`bt_navigator`):** Orchestrates high-level goal navigation via ROS 2 Behavior Trees (`NavigateToPose`).
- **Global Planner (`planner_server`):** Computes global paths across the environment map using A* search (`NavfnPlanner`).
- **Local Controller (`controller_server`):** Executes path tracking using **Regulated Pure Pursuit Controller**, featuring rotate-to-heading, velocity-scaled lookahead distance, and cost-regulated speed reduction.
- **Costmaps (`local_costmap` & `global_costmap`):** Dynamic 2D obstacle marking/clearing from RPLiDAR scans (`/scan`) combined with inflation cost scaling.
- **Velocity Smoother (`velocity_smoother`):** Filters and constrains velocity/acceleration ramps before passing commands to hardware interfaces.
- **Collision Monitor (`collision_monitor`):** Real-time safety layer enforcing active emergency stop bounding polygons (`PolygonStop` circle radius 0.17m) directly on scan sensor streams.
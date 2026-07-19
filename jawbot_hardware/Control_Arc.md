# Jawbot Autonomous Mobile Robot: Control Architecture Documentation

This document outlines the full-stack control architecture for the Jawbot AMR. The system uses a modular, decoupled approach that separates high-level ROS 2 kinematic planning from low-level deterministic motor control, bridged by a robust serial communication layer.

---

## Part 1: Microcontroller Firmware (Raspberry Pi Pico)

### 1.1 Methodology
The Pico acts as a real-time execution slave. The architecture uses Object-Oriented C++ principles, abandoning blocking functions (like `delay()`) in favor of hardware-timed interrupts. The methodology relies on a **50Hz Hardware Timer** to execute PID velocity math, paired with **Hardware Interrupt Service Routines (ISRs)** to capture quadrature encoder pulses instantly. It also includes **Deadband Compensation** ("Anti-Whine" logic) to smoothly start motors at low target velocities by skipping inactive PWM bands.

### 1.2 Files and Directory
| File Path | Description |
| :--- | :--- |
| `jawbot_pico.ino` | Main dispatcher, hardware timer setup, and communication heartbeat. |
| `L298N.h/.cpp` | Actuator physical layer (PWM and Direction logic). |
| `WheelEncoder.h/.cpp` | Sensor physical layer (Interrupt-driven quadrature tracking). |
| `gy87_driver.h/.cpp` | Sensor physical layer (I2C communication with GY-87 IMU module). |
| `pid.h/.cpp` | Mathematical engine for closed-loop velocity tracking. |
| `RobotJoint.h/.cpp` | Abstraction layer wrapping Motor, Encoder, and PID instances. |

### 1.3 Classes and Functions
| Class | Function | Description |
| :--- | :--- | :--- |
| **`L298N`** | `init()`, `setSpeed()` | Configures GPIOs and maps velocity to PWM signals. |
| **`WheelEncoder`**| `init()`, `handleInterrupt()`, `getTicks()` | Manages quadrature logic via ISRs and safe tick retrieval. |
| **`GY87`** | `init()`, `update()` | Initializes I2C bypass for MPU6050 and HMC5883L, and populates IMU struct with scaled accelerometer, gyroscope, and magnetometer data. |
| **`PIDController`**| `compute()`, `setGains()` | Executes velocity tracking math and supports live gain tuning. |
| **`RobotJoint`** | `executeControlLoop()` | Cascades logic: Encoder $\to$ Velocity Calc $\to$ PID $\to$ Deadband Skip $\to$ Motor PWM. |

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
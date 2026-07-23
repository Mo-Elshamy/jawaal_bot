# 🤖 Differential Drive Odometry Calibration Guide

Accurate odometry is the foundation of the entire ROS 2 Navigation Stack (Nav2). If the robot's physical movement does not perfectly match its software estimates, SLAM maps will warp, obstacles will duplicate, and local planners will fail. 

This guide outlines the 3-Phase standard operating procedure for calibrating a differential drive robot using the Ground Truth vs. Software Estimate method.

---

## 🛠 Prerequisites
* Robot is fully assembled with operating mass (battery, payload) attached.
* Low-level hardware interface (e.g., `ros2_control` + micro-controller) is running.
* PID velocity controllers are fully tuned.
* Access to a measuring tape and floor tape.

---

## 🟢 Phase 1: Base Encoder Calibration (Ticks Per Revolution)
**Objective:** Guarantee the micro-controller is perfectly converting raw magnetic pulses into standard radians.

### The Test (Suspended)
1. **Elevate the robot** so the drive wheels are suspended in the air (no ground friction).
2. Place a piece of tape on the wheel and a matching alignment mark on the chassis.
3. Open a terminal and monitor the raw joint states:
   ```bash
   ros2 topic echo /joint_states
   ```

4. Physically rotate the wheel by hand exactly **10 full revolutions** forward.

### The Math

* **Expected Value:** 10 revolutions × 2π = **62.8318 radians**.
* If the terminal reads negative (e.g., `-62.83`), the encoder Phase A/B wires are swapped. Physically swap them on the micro-controller.
* If the magnitude is wrong, calculate your true `ticks_per_rev` for your firmware:

> **Formula:**
> `Corrected Ticks Per Rev = Current Ticks Per Rev * (Actual Radians Read / 62.8318)`

**Action:** Flash the new integer to the micro-controller and repeat until 10 spins equals `~62.83` radians.

---

## 🔵 Phase 2: Linear Calibration (Effective Wheel Radius)

**Objective:** Correct for tire compression under the robot's weight.

### The Test (On the Floor)

1. Place the robot on its primary operating surface (carpet, tile, etc.).
2. Mark the starting position of the front bumper with tape.
3. Restart the ROS 2 bringup to zero out the odometry frame.
4. Launch a teleop node:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

5. Drive the robot perfectly straight forward.
6. Stop the robot, place a second piece of tape at the new bumper position, and measure the exact **Physical Distance** with a tape measure (e.g., `1.00 meters`).
7. Check what the software *thinks* happened:
   ```bash
   ros2 topic echo /odom --once
   ```

*Look at `pose.pose.position.x` for the **Software Distance**.*

### The Math

> **Formula:**
> `New Wheel Radius = Old Wheel Radius * (Physical Distance / Software Distance)`

**Action:** Update the `wheel_radius` parameter in your `controllers.yaml` file.

---

## 🟣 Phase 3: Angular Calibration (Track Width / Wheel Separation)

**Objective:** Correct for the true physical distance between the wheels, accounting for wheel thickness and turning friction.

### The Test (On the Floor)

1. Place a long piece of tape on the floor perfectly aligned with the left side of the robot.
2. Launch RViz to visualize the software's odometry frame:
   ```bash
   rviz2
   ```

*Set Fixed Frame to `odom` and add the RobotModel/TF display.*
3. Use the teleop node to spin the robot in place.
4. **Watch RViz, not the physical robot.** Spin until the *virtual* robot in RViz completes exactly **one 360-degree rotation** and faces perfectly straight again.
5. Look at the *physical* robot on the floor.
* Measure the physical angle it actually rotated relative to your starting tape line using a protractor.
* If it spun past the line, it over-rotated (e.g., 375°). If it stopped short, it under-rotated (e.g., 345°).

### The Math

> **Formula:**
> `New Track Width = Old Track Width * (Software Angle [360] / Physical Angle Measured)`

**Action:** Update the `wheel_separation` parameter in your `controllers.yaml` file.

---

## 🚨 Troubleshooting & Pro-Tips

* **Robot Drifts During Linear Test:** If the robot naturally curves while driving straight, your motors have mismatched friction. Increase the **Integral Gain (Ki)** in your PID controller so it aggressively forces the lagging wheel to match the target speed.
* **Command Velocity Watchdog:** If using a script instead of teleop to drive, remember to publish to `/cmd_vel` continuously at 10Hz. Publishing a single command and using `time.sleep()` will trigger the ROS 2 safety watchdog, and the robot will automatically brake after 0.5 seconds.
* **Recalibrate on New Surfaces:** Ground friction heavily influences track width. If moving a robot from smooth concrete to thick carpet, re-run Phase 3.

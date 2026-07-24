# 🤖 Jawbot Sensor & Odometry Calibration Guide

Accurate sensor readings and odometry are the foundation of the entire ROS 2 Navigation Stack (Nav2) and Extended Kalman Filter (EKF). If the robot's physical movement does not match its software estimates or if sensors drift while static, SLAM maps will warp, obstacles will duplicate, and state estimation will fail.

This guide outlines the standard operating procedures for calibrating both the **Differential Drive Odometry** and the **GY-87 IMU (MPU6050/HMC5883L)**.

---

## 📋 Table of Contents
1. [Part 1: Differential Drive Odometry Calibration](#-part-1-differential-drive-odometry-calibration)
   - [Phase 1: Base Encoder Calibration (Ticks Per Rev)](#-phase-1-base-encoder-calibration-ticks-per-revolution)
   - [Phase 2: Linear Calibration (Wheel Radius)](#-phase-2-linear-calibration-effective-wheel-radius)
   - [Phase 3: Angular Calibration (Track Width)](#-phase-3-angular-calibration-track-width--wheel-separation)
2. [Part 2: GY-87 IMU Calibration (MPU6050)](#-part-2-gy-87-imu-calibration-mpu6050)
   - [Step 1: Standalone Calibration Script](#step-1-run-the-standalone-calibration-script)
   - [Step 2: Applying Offsets to Firmware](#step-2-apply-the-offsets-to-your-architecture)
3. [Troubleshooting & Pro-Tips](#-troubleshooting--pro-tips)

---

# 🚜 Part 1: Differential Drive Odometry Calibration

## 🛠 Prerequisites
* Robot is fully assembled with operating mass (battery, payload) attached.
* Low-level hardware interface (`ros2_control` + micro-controller) is running.
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
5. Look at the *physical* robot on the floor:
   * Measure the physical angle it actually rotated relative to your starting tape line using a protractor.
   * If it spun past the line, it over-rotated (e.g., 375°). If it stopped short, it under-rotated (e.g., 345°).

### The Math
> **Formula:**  
> `New Track Width = Old Track Width * (Software Angle [360] / Physical Angle Measured)`

**Action:** Update the `wheel_separation` parameter in your `controllers.yaml` file.

---

# 🧭 Part 2: GY-87 IMU Calibration (MPU6050)

Calibrating the static offsets for the accelerometer and gyroscope makes a night-and-day difference in your ROS 2 Extended Kalman Filter (robot_localization). 

When the robot is sitting perfectly still:
* **Gyroscope** should report exactly `0.0 rad/s` on all axes.
* **Accelerometer** should report `0.0 m/s²` on X and Y, and `9.80665 m/s²` (1g of Earth's gravity) on Z.

Because of manufacturing tolerances, it never does. We calculate the difference between what the sensor *actually* reads and what it *should* read, and permanently subtract that error in the C++ driver.

---

### Step 1: Run the Standalone Calibration Script

Open a **brand new sketch** in the Arduino IDE and paste this self-contained script.

> ⚠️ **Critical Rule:** Place the robot on a perfectly flat, level floor. Once you click "Upload," **do not touch the robot or bump the table**. Even tiny vibrations will ruin the calibration.

```cpp
#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SCALE = 9.80665 / 16384.0;
const float GYRO_SCALE = 0.0174533 / 131.0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  Serial.println("DO NOT MOVE THE ROBOT.");
  Serial.println("Calibrating in 3 seconds...");
  delay(3000);
  Serial.println("Taking 2000 samples. Please wait...");

  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int num_samples = 2000;

  for (int i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 14, (int)true);

    if (Wire.available() == 14) {
      ax_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      ay_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      az_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      Wire.read(); Wire.read(); // Skip Temperature
      gx_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      gy_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      gz_sum += (int16_t)(Wire.read() << 8 | Wire.read());
    }
    delay(2); // 500Hz sampling rate
  }

  // Calculate raw averages converted directly to ROS standard units
  float ax_avg = ((float)ax_sum / num_samples) * ACCEL_SCALE;
  float ay_avg = ((float)ay_sum / num_samples) * ACCEL_SCALE;
  float az_avg = ((float)az_sum / num_samples) * ACCEL_SCALE;
  
  float gx_avg = ((float)gx_sum / num_samples) * GYRO_SCALE;
  float gy_avg = ((float)gy_sum / num_samples) * GYRO_SCALE;
  float gz_avg = ((float)gz_sum / num_samples) * GYRO_SCALE;

  // Calculate the offsets (Offset = Average Reading - Ideal Reading)
  float ax_off = ax_avg - 0.0;
  float ay_off = ay_avg - 0.0;
  float az_off = az_avg - 9.80665; // Z-axis should read Earth gravity
  
  float gx_off = gx_avg - 0.0;
  float gy_off = gy_avg - 0.0;
  float gz_off = gz_avg - 0.0;

  Serial.println("\n--- CALIBRATION COMPLETE ---");
  Serial.println("Copy and paste these lines into the private section of gy87_driver.h:\n");
  
  Serial.print("const float offset_accel_x = "); Serial.print(ax_off, 5); Serial.println(";");
  Serial.print("const float offset_accel_y = "); Serial.print(ay_off, 5); Serial.println(";");
  Serial.print("const float offset_accel_z = "); Serial.print(az_off, 5); Serial.println(";");
  
  Serial.print("const float offset_gyro_x = "); Serial.print(gx_off, 5); Serial.println(";");
  Serial.print("const float offset_gyro_y = "); Serial.print(gy_off, 5); Serial.println(";");
  Serial.print("const float offset_gyro_z = "); Serial.print(gz_off, 5); Serial.println(";");
}

void loop() {}
```

---

### Step 2: Apply the Offsets to Your Architecture

Once the serial monitor outputs your 6 generated offset values, embed them into the `jawbot_pico` firmware drivers:

**1. Update `gy87_driver.h`**  
Paste the generated lines into the `private:` section of your `GY87` class:

```cpp
class GY87 {
public:
    GY87();
    bool init();
    bool update(IMUData &data);

private:
    const uint8_t MPU_ADDR = 0x68;
    const uint8_t HMC_ADDR = 0x1E;

    const float ACCEL_SCALE = 9.80665 / 16384.0;
    const float GYRO_SCALE = 0.0174533 / 131.0;
    const float MAG_SCALE = 0.0000001; 

    // --- PASTE YOUR GENERATED OFFSETS HERE ---
    const float offset_accel_x = 0.12345; // (Example generated offset)
    const float offset_accel_y = -0.09876;
    const float offset_accel_z = 0.54321;
    const float offset_gyro_x = 0.00123;
    const float offset_gyro_y = -0.00456;
    const float offset_gyro_z = 0.00789;
};
```

**2. Update `gy87_driver.cpp`**  
Locate the conversion math inside `update()` and subtract the offsets:

```cpp
// Convert to ROS standard units AND apply calibration offsets
data.accel_x = (ax * ACCEL_SCALE) - offset_accel_x;
data.accel_y = (ay * ACCEL_SCALE) - offset_accel_y;
data.accel_z = (az * ACCEL_SCALE) - offset_accel_z;

data.gyro_x = (gx * GYRO_SCALE) - offset_gyro_x;
data.gyro_y = (gy * GYRO_SCALE) - offset_gyro_y;
data.gyro_z = (gz * GYRO_SCALE) - offset_gyro_z;
```

Re-flash your Jawbot Pico. Static readings will now report zero velocity/rotation on all axes when stationary.

---

# 🚨 Troubleshooting & Pro-Tips

* **Robot Drifts During Linear Test:** If the robot naturally curves while driving straight, your motors have mismatched friction. Increase the **Integral Gain (Ki)** in your PID controller so it aggressively forces the lagging wheel to match target speed.
* **Command Velocity Watchdog:** If using a script instead of teleop to drive, remember to publish to `/cmd_vel` continuously at 10Hz. Publishing a single command and using `time.sleep()` will trigger the ROS 2 safety watchdog, and the robot will automatically brake after 0.5 seconds.
* **Recalibrate on New Surfaces:** Ground friction heavily influences track width. If moving a robot from smooth concrete to thick carpet, re-run Phase 3 of Odometry Calibration.
* **IMU Mounting Orientation:** Ensure the GY-87 is mounted firmly and aligned with the robot's physical coordinate system (ROS 2 REP 103 standard: X forward, Y left, Z up).

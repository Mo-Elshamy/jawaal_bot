# 📊 EKF Tuning & Troubleshooting Guide (`robot_localization`)

This reference playbook provides guidance for tuning the Extended Kalman Filter (`ekf_filter_node`) parameters in `jawbot_localization` when observing unexpected robot state estimation, drift, or jitter.

---

## 🎯 High-Level Tuning Philosophy
EKF state estimation mathematically balances trust between physical kinematic models and live sensor payloads:
> *"Trust wheel encoders for forward velocity ($v_x$), trust the IMU gyroscope for angular rotation ($\omega_z$), and suppress stationary high-frequency sensor noise."*

---

## 1. Core Mathematical Tuning Parameters

### A. Process Noise Covariance Matrix ($Q$)
* **Location:** `jawbot_localization/config/ekf.yaml` under `process_noise_covariance`.
* **Description:** A $15 \times 15$ diagonal matrix representing uncertainty in the robot's physical motion model between timestep updates across 15 state variables:

$$\text{State} = [x, y, z, \text{roll}, \text{pitch}, \text{yaw}, v_x, v_y, v_z, \omega_{\text{roll}}, \omega_{\text{pitch}}, \omega_{\text{yaw}}, a_x, a_y, a_z]$$

```yaml
# Matrix order: x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
process_noise_covariance: [
  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  # ... (15x15 diagonal matrix)
]
```

* **High $Q$ Values:** Filter assumes physical motion model is uncertain $\rightarrow$ Relies heavily on incoming sensor measurements.
  * *Result:* Fast response to real movement, but higher jitter/noise in `/odom`.
* **Low $Q$ Values:** Filter assumes physical motion model is highly accurate $\rightarrow$ Relies strictly on model integration.
  * *Result:* Extremely smooth `/odom` data, but noticeable lag or overshoot during rapid acceleration/turns.

---

### B. Sensor Measurement Covariance ($R$)
* **Location:** Configured inside sensor drivers/broadcasters (e.g. `jawbot_bringup/config/jawbot_controllers.yaml` for `imu_broadcaster`).
* **Description:** Quantifies expected noise levels for raw sensor data streams.

```yaml
imu_broadcaster:
  ros__parameters:
    sensor_name: "imu_sensor"
    frame_id: "imu_link"
    static_covariance_angular_velocity: [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
    static_covariance_linear_acceleration: [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
```

* **Higher $R$ Value:** Filter treats sensor measurements as noisier, smoothing out spikes.
* **Lower $R$ Value:** Filter treats sensor data as high precision, trusting raw values immediately.

---

### C. Outlier Rejection Thresholds (`*_reject_threshold`)
* **Location:** `jawbot_localization/config/ekf.yaml`.
* **Description:** Safety gates based on Mahalanobis Distance (statistical distance of new measurements from expected state).

```yaml
odom0_reject_threshold: 5.0
imu0_reject_threshold: 3.0
```

* **How it works:** If a sensor measurement spikes unexpectedly (e.g. electrical noise or wheel bump), the EKF rejects the sample entirely instead of corrupting the robot's pose estimate.

---

## 2. Practical Tuning & Symptom Matrix

| Symptom | Root Cause | Recommended Fix |
|---|---|---|
| **`/odom` pose is jittery/shaking when stationary** | Process noise ($Q$) set too high or sensor noise covariance ($R$) set too low. | • Decrease $Q$ for $v_x$ and $\omega_z$ in `ekf.yaml`.<br>• Increase static covariance ($R$) in `jawbot_controllers.yaml`. |
| **Robot turns in real life, but `/odom` orientation lags behind** | Filter is forcing the robot to follow the linear motion model too strictly. | • Increase $Q$ value for yaw velocity ($\omega_z$) or yaw acceleration ($a_z$) in `ekf.yaml`. |
| **Wheels slip on smooth floors causing position drift** | Filter relies excessively on wheel speed ($v_x$) during slippage. | • Decrease $Q$ for wheel speed $v_x$.<br>• Increase reliance on IMU $x$-acceleration ($a_x$). |
| **Sudden position jump/teleporting in RViz** | Spurious sensor artifact passed into the filter without rejection. | • Lower `imu0_reject_threshold` or `odom0_reject_threshold` (e.g., from `5.0` to `3.0`). |

---

## 3. Quick-Reference Checklist for Odd Behavior

1. **Verify Static State:** Stop the robot. Run `ros2 topic echo /odom`. Position and orientation should not drift or jitter.
2. **Verify Gyro Yaw Response:** Rotate the robot by hand $90^\circ$. Check if `/odom` orientation updates in real-time without delay.
3. **Verify Rejection Logs:** Check terminal output of `ekf_node` for warnings regarding rejected measurements.

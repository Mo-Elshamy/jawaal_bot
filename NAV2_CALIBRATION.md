# Nav2 Parameter & Calibration Guide (`nav2_params.yaml`)

This guide explains the parameters in `jawbot_navigation/config/nav2_params.yaml`, their physical effects on the **Jawbot AMR**, and common real-world calibration scenarios with recommended solutions.

---

## 1. Parameter Overview & Effects

### 1. AMCL (`amcl`)
| Parameter | Default Value | Function & Physical Effect |
| :--- | :--- | :--- |
| `min_particles` / `max_particles` | `500` / `2000` | Controls particle filter population density. Higher numbers increase localization resilience during aggressive turns but increase CPU usage. |
| `alpha1` to `alpha4` | `0.2` | Expected noise parameters in differential motion model (rotation from rotation, rotation from translation, translation from translation, translation from rotation). Higher values increase particle spread during movement. |
| `update_min_d` / `update_min_a` | `0.1m` / `0.1 rad` | Minimum linear displacement or rotation required to trigger a particle filter update step. Keeps localization idle while stationary. |
| `laser_model_type` | `likelihood_field` | Standard scan matching model. Fast and robust for indoor lidar mapping. |
| `z_hit` / `z_rand` | `0.8` / `0.05` | Weight given to scan hits vs. random noise scans. |

### 2. Local Controller (`controller_server` & `RegulatedPurePursuitController`)
| Parameter | Default Value | Function & Physical Effect |
| :--- | :--- | :--- |
| `desired_linear_vel` | `0.25 m/s` | Maximum target cruising speed along paths. |
| `lookahead_dist` | `0.4 m` | Standard distance along global path where the pure pursuit controller places its target point. |
| `min_lookahead_dist` / `max_lookahead_dist` | `0.2m` / `0.6m` | Bounds for dynamic lookahead scaling. Shorter lookaheads tighten cornering; longer lookaheads smooth out straight paths. |
| `use_rotate_to_heading` | `true` | Enables turning-in-place when starting a path or reaching a goal if heading error is large. Prevents wide curved arcs when initiating movements. |
| `rotate_to_heading_min_angular_vel` | `1.2 rad/s` | Minimum angular speed threshold during rotate-to-heading to overcome wheel stiction and ground friction. |
| `use_cost_regulated_linear_velocity_scaling` | `true` | Automatically slows down linear speed when operating close to high-cost inflation zones (near obstacles). |

### 3. Global Planner (`planner_server` & `NavfnPlanner`)
| Parameter | Default Value | Function & Physical Effect |
| :--- | :--- | :--- |
| `use_astar` | `true` | Switches planner search algorithm from Dijkstra to A*. Reduces path computation latency. |
| `allow_unknown` | `true` | Allows global path planning through unmapped/unexplored space. |
| `tolerance` | `0.2m` | Distance tolerance around requested goal position to find a valid free cell if goal is blocked. |

### 4. Costmaps (`local_costmap` & `global_costmap`)
| Parameter | Default Value | Function & Physical Effect |
| :--- | :--- | :--- |
| `robot_radius` | `0.15m` | Circular collision boundary around robot base center (`base_footprint`). |
| `inflation_radius` | `0.35m` | Buffer zone around obstacles where traversal cost decreases exponentially outwards. |
| `cost_scaling_factor` | `3.0` | Exponential decay rate of inflation cost. Higher values cause sharp cost drop-offs near obstacles; lower values create wider safety corridors. |
| `raytrace_max_range` / `obstacle_max_range` | `3.0m` / `2.5m` (Local) | Range bounds for clearing vs. inserting obstacle points into local costmap from RPLiDAR scans. |

### 5. Velocity Smoother (`velocity_smoother`)
| Parameter | Default Value | Function & Physical Effect |
| :--- | :--- | :--- |
| `max_velocity` | `[0.25, 0.0, 1.5]` | Max linear velocity ($X$), lateral velocity ($Y$), and angular velocity ($Z$). |
| `min_velocity` | `[-0.10, 0.0, -1.5]` | Min reverse velocity and angular limits. |
| `max_accel` / `max_decel` | `[0.5, 0.0, 3.2]` / `[-0.5, 0.0, -3.2]` | Acceleration and deceleration constraints ($m/s^2$ and $rad/s^2$) to prevent wheel slip and mechanical jerks. |

### 6. Collision Monitor (`collision_monitor`)
| Parameter | Default Value | Function & Physical Effect |
| :--- | :--- | :--- |
| `cmd_vel_in_topic` / `cmd_vel_out_topic` | `cmd_vel_smoothed` $\to$ `cmd_vel` | Intercepts smoothed velocity commands and halts output if LIDAR scan enters safety polygon. |
| `PolygonStop.radius` | `0.17m` | Emergency stop circle radius surrounding the 0.15m footprint. |
| `min_points` | `3` | Number of LIDAR range hits inside the polygon required to trigger an immediate zero-velocity halt. |

---

## 2. Common Calibration Situations & Recommendations

### Situation 1: The Robot Oscillates or Bounces Back and Forth Along Narrow Hallways
- **Symptom:** The robot sways side-to-side repeatedly or struggles to follow a straight path.
- **Cause:** Lookahead distance is too short, or inflation costs drop off too slowly.
- **Recommended Tuning:**
  1. Increase `lookahead_dist` from `0.4` to `0.5m` or `0.6m`.
  2. Increase `cost_scaling_factor` in both costmaps from `3.0` to `5.0` to steepen cost falloff near walls.

### Situation 2: The Robot Fails to Rotate in Place When Navigating to a New Goal
- **Symptom:** The robot whines or stalls when attempting to turn in place towards a path segment.
- **Cause:** `rotate_to_heading_min_angular_vel` is lower than the physical ground friction threshold.
- **Recommended Tuning:**
  1. Increase `rotate_to_heading_min_angular_vel` in `controller_server` (e.g., from `1.2` to `1.4` or `1.5 rad/s`).
  2. Ensure the firmware dual-stage friction compensation (`STATIC_BREAKAWAY` in `RobotJoint.cpp`) is set appropriately.

### Situation 3: Sudden Emergency Stops Near Doorways or Tight Spaces
- **Symptom:** Robot freezes near narrow door frames even though global planner showed a valid path.
- **Cause:** Inflation radius is too large or collision monitor polygon footprint (`PolygonStop`) is oversized for the clearance width.
- **Recommended Tuning:**
  1. Reduce `inflation_radius` from `0.35m` to `0.25m` - `0.30m`.
  2. Verify `PolygonStop.radius` in `collision_monitor` is set just outside the robot radius (e.g. `0.17m` for `0.15m` robot radius).

### Situation 4: AMCL Particles Spread Out and Robot Loses Map Tracking During Fast Turns
- **Symptom:** In RViz, laser scans mismatch map walls after rapid rotational maneuvers.
- **Cause:** Motion noise parameters (`alpha1` / `alpha2`) are underestimating actual wheel slippage, or AMCL update rate is too slow.
- **Recommended Tuning:**
  1. Increase `alpha1` (rotational noise from rotation) from `0.2` to `0.3`.
  2. Decrease `update_min_a` in AMCL from `0.1` to `0.05 rad` for higher frequency orientation updates.

### Situation 5: Mechanical Jerking and Motor Tripping on Acceleration
- **Symptom:** Current spikes occur or motor driver resets during rapid speed transitions.
- **Cause:** Acceleration limits in `velocity_smoother` exceed motor drive torque limits.
- **Recommended Tuning:**
  1. Reduce `max_accel` in `velocity_smoother` from `0.5` to `0.3 m/s²`.
  2. Reduce angular acceleration `max_accel[2]` from `3.2` to `2.0 rad/s²`.

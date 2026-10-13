# Handoff 2 — Leroy ROS 2 Jazzy: Changes Required for XRSLAM Integration

## 1. Mission

Update Leroy's existing ROS 2 Jazzy packages so that **XRSLAM becomes the exclusive odometry and localization provider**, while Leroy retains responsibility for hardware, static robot description and system orchestration.

The architecture must keep Nav2 and XRSLAM as independent ROS 2 packages.

**Leroy ROS workspace:** `/home/kevin/Code/leroy/ros`  
**ROS distribution:** Jazzy  
**Source basis:** User-provided description of the checked-in packages. The live ROS graph has not yet been inspected.

Existing packages:

| Package | Responsibility |
|---|---|
| `leroy_msgs` | Custom MCU encoder message |
| `leroy_bridge` | MCU serial communication, IMU/encoder telemetry, motor commands |
| `leroy_description` | Robot URDF and fixed transforms |
| `leroy_bringup` | Description, bridge, camera, rectification and DepthART launch orchestration |

**Do not implement XRSLAM functionality inside these Leroy packages.** Prepare them to work with an independent `xrslam_ros2` package.

## 2. Final interface ownership

| Function | Owner |
|---|---|
| Camera/IMU mounting geometry | `leroy_description` |
| Static sensor transforms | `robot_state_publisher` using Leroy's URDF |
| Camera intrinsics | Camera calibration file, published as `CameraInfo` |
| Raw IMU measurements | `leroy_bridge` |
| Raw wheel encoder counts | `leroy_bridge` |
| Serial communication with MCU | `leroy_bridge` |
| Motor commands | `leroy_bridge` |
| Visual-inertial odometry | `xrslam_ros2` |
| Dynamic `odom → base_link` | **`xrslam_ros2` exclusively** |
| Persistent `map → odom` | Future verified XRSLAM global-localization interface |
| Semantic map and class queries | XRSLAM |
| Path planning and collision avoidance | Nav2 |
| Live depth point cloud | Camera/depth pipeline, not necessarily XRSLAM |
| Global 2D navigation occupancy map | Separate persistent map provider |

No package should duplicate another package's dynamic TF ownership.

## 3. `leroy_bridge` changes

### 3.1 Disable degraded wheel odometry

File:

`ros/src/leroy_bridge/config/leroy_bridge.yaml`

Change:

- `degraded_odometry_enabled: false`
- `publish_odom_tf: false`

The bridge's current degraded odometry uses the front-left/rear-left encoders and IMU yaw. Some encoders are faulty, so it must not be the authoritative odometry source.

Verify the implementation stops publishing:

- `/odom`
- `odom → base_link`

when the relevant settings are disabled.

Keep the degraded estimator's source code available for future diagnostics or an explicitly selected fallback mode. **Do not activate a fallback automatically**, since that could introduce competing publishers or discontinuous odometry.

### 3.2 Preserve hardware interfaces

The bridge must continue:

- Publishing `/imu/data_raw` as `sensor_msgs/msg/Imu`, best effort, depth 10.
- Publishing `/wheel/encoder_counts` as `leroy_msgs/msg/EncoderCounts`, best effort, depth 10.
- Subscribing to `/cmd_vel` as `geometry_msgs/msg/Twist`, reliable, depth 10.
- Sending motor commands and telemetry requests over serial.
- Retaining the MCU watchdog and command-timeout safety behavior.

The raw encoder topic may remain available even though XRSLAM will ignore it initially.

The IMU contract is:

- Frame: `imu_link`
- Nominal rate: 100 Hz
- Axes: +X forward, +Y left, +Z up
- Angular velocity: rad/s
- Linear acceleration: m/s²
- Orientation estimate unavailable, expressed by `orientation_covariance[0] = -1`

XRSLAM must use angular velocity and acceleration without expecting a quaternion orientation from the bridge.

### 3.3 Preserve the motor safety interlock

Current configuration:

`max_wheel_angular_velocity_rad_s: 0.0`

This means **motor output is disabled**.

Do not increase this value for XRSLAM or Nav2 testing without a separate physical-robot safety review and explicit authorization.

Also preserve:

- `watchdog_ms: 500`
- `keepalive_period_s: 0.10`
- `cmd_vel_timeout_s: 0.25`

The bridge checks command timeout periodically and stops motors on shutdown. Keep those behaviors independent of Nav2's Collision Monitor.

### 3.4 Preserve wheel geometry for motor control

The bridge still needs:

- `wheel_radius_m: 0.020`
- `half_wheelbase_m: 0.180`
- `half_track_m: 0.275`

These values remain relevant to mecanum inverse kinematics and motor commands **even when wheel odometry is disabled**.

Wheel geometry is not required by XRSLAM's camera/IMU VIO.

To avoid long-term configuration duplication, consider storing canonical physical wheel dimensions in shared robot-description parameters and generating the bridge's motion configuration from them. This is a later cleanup, not a prerequisite for initial XRSLAM integration.

Do not remove the geometry parameters from the bridge before its motor-control path has an equivalent source.

### 3.5 Check timestamp behavior

The bridge translates MCU microsecond timestamps into ROS timestamps using a clock offset, including uint32 wrap handling.

The XRSLAM integration requires consistent camera/IMU timebases. Validate:

- Strictly increasing IMU timestamps.
- No unexpected discontinuities after MCU counter wrapping or reconnecting.
- Correct ROS/system/simulation time mode.
- Realistic camera/IMU timestamp offsets.
- No large scheduling delay interpreted as a measurement timestamp.

Retain existing sequence-gap diagnostics.

## 4. `leroy_description` changes

### 4.1 Keep the URDF as the single source of static sensor geometry

File:

`ros/src/leroy_description/urdf/leroy.urdf.xacro`

The URDF must remain authoritative for:

- `base_link → imu_link`
- `base_link → front_camera_link`
- `front_camera_link → front_camera_optical_frame`
- `front_camera_optical_frame → camera_optical_frame`
- `base_link → workspace_camera_link`
- `workspace_camera_link → workspace_camera_optical_frame`

`robot_state_publisher` should publish the corresponding fixed transforms.

**Do not make `leroy_bridge` separately broadcast these static transforms.** Doing so would duplicate TF ownership.

### 4.2 Validate the actual physical calibration

Known current values:

- IMU translation: `(0.095, -0.095, 0)` m.
- Front camera: approximately 30° downward tilt.
- Workspace camera: approximately 60° downward tilt.
- Camera translations and IMU height are currently approximated as zero.
- Camera optical frame: +X right, +Y down, +Z forward.

The approximate values are not sufficient for calibrated visual-inertial estimation.

Required work:

1. Measure or calibrate front-camera and IMU positions.
2. Confirm the sensor's actual rotational alignment.
3. Ensure the camera optical frame follows the ROS optical convention.
4. Confirm that the transform between `base_link` and `imu_link` matches the IMU axis contract.
5. Confirm that `camera_optical_frame` genuinely identifies the rectified camera image frame.
6. Update the canonical URDF/Xacro values.

The XRSLAM node will read these values through TF2 at startup and translate them into its internal camera/IMU extrinsics.

Avoid maintaining independent copies of the mounting transforms in XRSLAM device YAML.

### 4.3 Keep static and dynamic TF separate

The intended tree is:

`map → odom → base_link → sensor frames`

Ownership:

- `map → odom`: XRSLAM, only after verified global localization.
- `odom → base_link`: XRSLAM VIO adapter.
- `base_link → imu_link` and camera links: `robot_state_publisher`.

Do not add a fixed `odom → base_link` joint to the URDF.

Do not publish a fake identity `map → odom` as evidence of successful persistent-map localization.

## 5. `leroy_bringup` changes

Main file:

`ros/src/leroy_bringup/launch/robot.launch.py`

Current bringup already starts the description and bridge, with optional camera and DepthART components.

### 5.1 Add an optional XRSLAM launch integration

Introduce an explicit launch argument such as:

`use_xrslam`, initially defaulting to `false` until sensor synchronization and TF behavior have been tested.

When enabled, include or launch the separate `xrslam_ros2` package.

Pass parameters or remappings for:

- IMU topic `/imu/data_raw`
- Rectified front-camera image topic
- Rectified camera calibration topic
- Robot body frame `base_link`
- IMU frame `imu_link`
- Camera frame `camera_optical_frame`
- Odometry frame `odom`

Avoid embedding XRSLAM estimator settings into `leroy_bringup`.

The XRSLAM node itself should wait for its required TF and `CameraInfo` inputs before creating the estimator. A fixed launch sleep is not sufficient.

### 5.2 Preserve launch independence

Existing arguments include:

- `use_camera`, default `true`
- `use_depthart`, default `true`
- `camera_name`, default `front_camera`
- `camera_frame`, default `camera_optical_frame`
- Camera dimensions 640 × 480
- Frame rate 30 Hz
- Camera calibration file `/config/camera/front_camera.yaml`

Keep these configurable.

The bridge and robot description should remain usable without XRSLAM. XRSLAM should remain usable for replay/testing without Nav2.

### 5.3 Verify rectified camera topics

The bringup configures a camera driver and `image_proc::RectifyNode`, but the exact live topics and QoS must be confirmed.

Do not assume `/camera/image_rect` exists without checking.

Validate using `ros2 topic list`, `ros2 topic info --verbose`, and `ros2 topic echo`.

Ensure XRSLAM receives:

- A rectified image.
- Matching `CameraInfo`.
- Correct image dimensions and encoding.
- Consistent timestamps.
- The correct optical-frame identifier.

Avoid duplicate undistortion inside the XRSLAM adapter.

### 5.4 Keep DepthART optional and separate

The current launch delegates DepthART behavior to the external `depthart_ros` package, obtained from an unpinned `main` branch.

Its exact output topics, depth units, registration and metric calibration are not yet established from this repository.

Before wiring DepthART into Nav2's Voxel Layer, verify:

- Published topic names and message types.
- Whether the model predicts metric or relative depth.
- Units and scaling.
- Valid range and uncertainty.
- RGB/depth pixel registration.
- Point-cloud generation and TF frame correctness.

**Do not use unvalidated monocular depth for safety-critical collision avoidance.**

The initial XRSLAM VIO integration can proceed using camera images and IMU without DepthART.

## 6. `leroy_msgs` changes

No changes are currently required.

Keep:

`leroy_msgs/msg/EncoderCounts`

with fields:

- `std_msgs/Header header`
- `uint32 sequence`
- `int64 front_left`
- `int64 front_right`
- `int64 rear_left`
- `int64 rear_right`

Do not alter the wire-level MCU encoder meaning or synthesize missing right-encoder values.

A future XRSLAM semantic-query service should belong to an XRSLAM-specific interface package, not `leroy_msgs`, because semantic search is not a Leroy hardware contract.

## 7. Nav2 interoperability — without a direct dependency

Do not add Nav2 dependencies to the four Leroy packages merely to connect them.

The standard interface will be:

| Source | Destination | Contract |
|---|---|---|
| XRSLAM | Nav2 | `/odom` and TF |
| Camera/depth pipeline | Nav2 local costmap | Metric `PointCloud2` |
| Separate persistent map provider | Nav2 global costmap | `OccupancyGrid` |
| Nav2 controller pipeline | Leroy bridge | Final `/cmd_vel` |

For eventual navigation:

- Use Nav2's Voxel Layer and Inflation Layer for live 3D obstacle processing and collision margins.
- Use a separately maintained persistent 2D map for global path planning.
- Ensure a consistent `map → odom → base_link` frame chain.
- Use a velocity-smoothing/collision-monitoring chain that has **one final publisher** to `/cmd_vel`.
- Do not bypass collision monitoring when routing Nav2 commands to the bridge.

The XRSLAM semantic query API will return historical observation poses. An independent application can later submit a chosen pose to Nav2's `NavigateToPose` action.

There should be **no direct XRSLAM-to-Nav2 API call**.

## 8. Bringup validation checklist

Carry out the following in stages, initially with motor output disabled.

### Stage A — Static geometry

- Confirm `robot_state_publisher` publishes `/tf_static`.
- Verify `base_link → imu_link` using TF2.
- Verify `base_link → camera_optical_frame`.
- Compare measured physical sensor positions with URDF values.
- Confirm only one publisher provides each fixed transform.

Useful commands:

- `ros2 run tf2_ros tf2_echo base_link imu_link`
- `ros2 run tf2_ros tf2_echo base_link camera_optical_frame`

### Stage B — Hardware inputs

- Verify `/imu/data_raw` at approximately 100 Hz.
- Check best-effort QoS compatibility.
- Verify image and camera calibration topics.
- Check IMU and image timestamp monotonicity and alignment.
- Verify the bridge continues to receive `/cmd_vel` while motors remain disabled.

### Stage C — Odometry ownership

With degraded bridge odometry disabled and XRSLAM running:

- Confirm exactly one publisher on `/odom`.
- Confirm exactly one dynamic `odom → base_link` authority.
- Check odometry values are finite and timestamps advance.
- Check body-frame orientation and translation directions.
- Check no unexpected jumps occur on tracking restart or relocalization.

Useful commands:

- `ros2 topic info --verbose /odom`
- `ros2 run tf2_ros tf2_echo odom base_link`
- `ros2 topic echo /xrslam/tracking_status`

### Stage D — Navigation readiness

Before allowing motor commands:

- Verify XRSLAM can initialize and track reliably on the real robot.
- Verify tracking loss is reported and causes navigation to inhibit motion.
- Verify the persistent global-map alignment is established.
- Verify a valid global 2D occupancy map is available.
- Verify metric depth obstacle detection with real measured distances.
- Test Nav2 path planning, obstacle response and command routing without actuation.
- Review and explicitly authorize motor-enable parameters.

## 9. Acceptance criteria

The Leroy changes are complete when:

1. The bridge still supplies raw IMU, encoder telemetry and motor commands.
2. The bridge no longer publishes `/odom` or `odom → base_link`.
3. `leroy_description` remains the sole static sensor-transform authority.
4. The independent XRSLAM ROS 2 node initializes from actual TF and `CameraInfo`.
5. XRSLAM exclusively publishes the dynamic odometry interface.
6. No wheel-encoder input is required for XRSLAM to operate.
7. Camera/IMU timestamps and coordinate frames are validated.
8. All packages build under ROS 2 Jazzy.
9. Existing Leroy hardware safeguards remain intact.
10. No Nav2 dependency has been introduced into the bridge or XRSLAM adapter.

**Current task status:** These are required changes, not a claim that the Leroy checkout has already been modified. The user-provided contract describes checked-in behavior; the actual live ROS graph, DepthART outputs and hardware calibration still require verification.

**Implementation priority:** First disable competing bridge odometry and validate canonical sensor TF. Next enable the standalone XRSLAM Jazzy adapter. Only then proceed to persistent mapping and independent Nav2 integration.
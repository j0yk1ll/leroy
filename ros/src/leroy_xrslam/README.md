# leroy_xrslam — Jazzy bootstrap adapter

Leroy-owned **ROS 2** / `ament_cmake` package, maintained in `leroy/ros/src/leroy_xrslam`. The standard Leroy Docker image does not build this optional adapter until the independent `libxrslam.so` has been built. It intentionally does not use
`xrslam-ros` (ROS 1), RTAB-Map, Nav2 libraries, wheel encoders, or occupancy
mapping. Only the XRSLAM C API is linked; do not add this package to the
XRSLAM superbuild.

**Status: compiled and linked successfully in a ROS 2 Jazzy Podman test on
2026-10-13. Robot-specific tracking, calibration, pose-direction and
loss/recovery behavior are not validated. Keep motors disabled.**

For a repeatable image build and safe smoke tests, see
[`ros/XRSLAM_CONTAINER.md`](../../XRSLAM_CONTAINER.md).

## Prerequisites

1. Build XRSLAM independently, including its existing C API (`libxrslam.so`).
2. Source a ROS 2 Jazzy environment with `ament_cmake`, `rclcpp`,
   `tf2_ros`, the common ROS messages, and `yaml-cpp`.
3. Obtain a **robot-calibrated** SLAM YAML and sensor/noise template YAML.
   The EuRoC example sensor file is NOT a substitute for Leroy's noise or
   temporal calibration. The sole YAML-driven `leroy_description` static TF launch owns sensor extrinsics; the URDF/robot_state_publisher path was retired in Stage 3's source candidate. No duplicate static broadcaster may run.
4. Use `ros2 topic list -t`, `ros2 topic info -v /imu/data_raw`, and
   `ros2 topic info -v <candidate-image-topic>` to identify the actual
   rectified image and matching CameraInfo topics. Both must use the optical
   frame ID that is in the TF tree.

Example workspace (adjust paths):

```sh
source /opt/ros/jazzy/setup.bash
mkdir -p ~/xrslam_ws/src
ln -s /path/to/leroy/ros/src/leroy_xrslam ~/xrslam_ws/src/leroy_xrslam
cd ~/xrslam_ws
colcon build --packages-select leroy_xrslam --cmake-args \
  -DXRSLAM_ROOT=/home/kevin/Code/xrslam \
  -DXRSLAM_LIBRARY=/home/kevin/Code/xrslam/lib/libxrslam.so
source install/setup.bash
export LD_LIBRARY_PATH=/home/kevin/Code/xrslam/lib:"${LD_LIBRARY_PATH}"
```

Choose parameters explicitly; no placeholder transforms or guessed time offsets:

```sh
ros2 run leroy_xrslam leroy_xrslam_node --ros-args \
  -p slam_yaml:=/absolute/path/leroy_slam.yaml \
  -p device_template_yaml:=/absolute/path/leroy_sensor_calibrated.yaml \
  -p camera_imu_offset_sec:=0.0 \
  -p image_topic:=/actual/rectified/image/topic \
  -p camera_info_topic:=/matching/camera_info/topic \
  -p camera_optical_frame:=camera_optical_frame \
  -p imu_frame:=imu_link
```

The `0.0` offset above is an **example only** for the visual-inertial
pipeline and must not be treated as a measured calibration. The SLAM YAML's
`sensor.mode` is the mode selector:

- **`monocular`**: camera-only diagnostic. Omit
  `camera_imu_offset_sec` (or explicitly use 0); no IMU ROS subscription,
  IMU TF lookup, gyro/acceleration forwarding, or wait-for-IMU camera gating.
  The system still needs `base_link <- camera_optical_frame` TF and matched,
  rectified image + CameraInfo messages. Because monocular trajectory scale is
  unobservable, `pose_convention_validated=true` and `enable_map_tf=true`
  are rejected; no metric odometry, pose or map TF is published. Use
  `/xrslam/tracking_status` and `/xrslam/localization_valid` as diagnostics.
- **`monocular_imu`**: visual-inertial mode. A finite,
  independently measured `camera_imu_offset_sec` is required by the
  deployment contract; `base_link <- imu_link` TF, gyro/acceleration stream
  and the offset-aware first-forwarded-IMU camera gate are active. For a
  stationary software diagnostic only, a clearly labelled provisional value
  may be passed without claiming calibration acceptance.

Both paths use the **P matrix** from CameraInfo and disable XRSLAM distortion
for rectified input. Camera frames received before TF-dependent library
initialization are discarded (not buffered). Once initialized, the bounded
camera queue and latched overflow fault remain active.

ROS outputs: `/xrslam/tracking_status` (String),
`/xrslam/localization_valid` (latched Bool),
`/xrslam/pose` (PoseStamped), `/odom` (Odometry) and
`odom -> base_link` (dynamic TF), *only after safety gates pass*.

By default `pose_convention_validated=false`: the node **does not publish
odometry** before the nonidentity-transform pose-direction test is performed.
Only after verifying on the robot that `XRSLAM_RESULT_BODY_POSE` represents
the robot body in the local VIO frame should that parameter be set to true.
Velocity is finite-differenced from consecutive body poses, **not directly
measured**. Covariance is deliberately conservative/uninformative (variance
floors of 100 in SI units), not estimator-derived; calibrate uncertainty and
velocity before motion control. There must be exactly one publisher of
`odom -> base_link`.

On tracking failure, clock/IMU discontinuity or a pose jump, odometry output is
**latched off** until an externally verified session alignment or a supervised
full node restart; this first adapter intentionally does not claim automatic
continuity-preserving recovery. No `map -> odom` is published.

## Safe two-stage integration check

Keep Leroy stationary with wheel power physically inhibited. Build the
optional XRSLAM image from the corresponding source branch.

1. Start with an otherwise unchanged copy of the XRSLAM SLAM YAML containing
   `sensor: {mode: monocular}`. Run the sidecar using
   `/camera/image_rect` and `/camera/camera_info`, without an IMU topic or
   temporal offset requirement. Keep
   `pose_convention_validated=false` and `enable_map_tf=false`.
   Confirm library startup, `initializing` status (or
   `pose_convention_unverified` if visual tracking succeeds), and zero
   odometry messages. Stationary scenes and pure yaw rotations might not
   provide sufficient triangulation parallax for monocular initialization.
2. Repeat with `sensor: {mode: monocular_imu}`, the real `/imu/data_raw`
   topic, and a **labelled provisional temporal offset for diagnostics only**
   until a hardware-calibrated value is available. Expect the same odometry
   inhibition. Compare tracking/initialization state and failures, not
   unvalidated metric trajectory accuracy.

No autonomous motion, calibration override, or enabling of map/odom TF is
part of these checks.

## Acceptance work still required

- Jazzy replay test with synchronized camera/IMU
- Nonidentity quaternion and translation fixture verifying body pose and TF
  directions (`q_bc`, `p_bc`, `q_bi`, `p_bi`)
- Hardware-measured timestamp offsets, time-domain/clock reset handling
- Validate Euler/quaternion and ROS `odom` child-frame twist conventions
- Verify tracking-loss latch and continuity over synthetic session restart
- Check TF authority and no competing `odom -> base_link` publisher
- EuRoC healthy/dropout regression and semantic/dense reconstruction parity

This package does not implement persistent-map save/load, semantic-keyframe
search, global `map -> odom`, Nav2 occupancy-grid generation, or navigation.

# Leroy ↔ XRSLAM (ROS 2 Jazzy) integration / verification

**Status:** Code integration only. No claim of Jazzy compilation, live ROS
graph inspection, physical camera/IMU calibration, tracking continuity or
navigation safety testing. The motor output safety interlock stays at 0.0.

## Package boundaries

- Leroy: MCU serial, raw `/imu/data_raw` and `/wheel/encoder_counts`,
  `/cmd_vel`, URDF fixed sensor TF, USB camera + rectifier, optional DepthART.
- Standalone [xrslam_ros2](https://github.com/j0yk1ll/xrslam/tree/main/xrslam_ros2):
  startup TF2 and `CameraInfo` calibration, VIO, exclusive dynamic
  `odom -> base_link`, `/odom`, tracking status.
- Future verified global localization: `map -> odom`; a separate map server
  provides persistent 2D `OccupancyGrid` for Nav2. No fake map TF.
- Nav2 remains independent, and is **not** a dependency of Leroy or XRSLAM.
  Metric obstacle clouds require real-world validation before use for safety.
  Route its final `/cmd_vel` through a single collision-monitor pipeline.

Remove any locally installed in-tree Leroy `xrslam_ros2` package before
building an overlay. Keep the package from the XRSLAM fork as the **only**
`xrslam_ros2` in your ROS environment. The Leroy Dockerfile intentionally
does not build or vendor XRSLAM; install the standalone package and its
`libxrslam.so` into the same container/runtime if using Docker.

## Build / source

```bash
source /opt/ros/jazzy/setup.bash
cd /home/kevin/Code/leroy/ros
colcon build --packages-select leroy_msgs leroy_bridge leroy_description leroy_bringup
source install/setup.bash

# In a separately built and sourced XRSLAM overlay:
ros2 pkg prefix xrslam_ros2
```

Follow the XRSLAM repository's `xrslam_ros2/README.md` to build the XRSLAM
C API and Jazzy adapter with `XRSLAM_ROOT` and `XRSLAM_LIBRARY`.
Supply a real calibrated device-template YAML (sensor noise, etc.) and a
separate monocular/IMU SLAM YAML; do not reuse the old Leroy-owned XRSLAM
copy or its EuRoC example noise values.

## Staged bringup (motors still disabled)

1. **Static TF:** `ros2 run tf2_ros tf2_echo base_link imu_link`;
   `ros2 run tf2_ros tf2_echo base_link camera_optical_frame`.
   Confirm just one authority for every fixed sensor transform and measure
   physical positions/rotations. Update `leroy.urdf.xacro` when calibrated.
2. **Sensor graph:** Run `ros2 launch leroy_bringup robot.launch.py
   use_depthart:=false`, then inspect `ros2 topic list -t`,
   `ros2 topic info --verbose /imu/data_raw`,
   `ros2 topic info --verbose /camera/camera_info`, and the *actual*
   rectified image topic. Do **not** assume `/camera/image_rect` exists.
   Validate matching frame IDs, dimensions, encoding and P intrinsics
   (`CameraInfo` for rectified monocular image), and sensor-data
   best-effort QoS. Verify IMU near 100 Hz:
   `ros2 topic hz /imu/data_raw`.
3. **Timebase:** Record raw IMU and image headers. Verify strictly increasing
   per-stream stamps, MCU uint32 wrap and disconnect/reconnect behavior,
   system vs simulation time mode, sensor capture latency, and a physically
   measured camera–IMU offset. A discontinuous clock requires supervised
   estimator restart/revalidation, **not** automatic fallback.
4. **VIO start:** With XRSLAM in the same ROS environment and measured inputs,
   explicitly launch (substitute observed topic names and verified files):

   ```bash
   ros2 launch leroy_bringup robot.launch.py \
     use_depthart:=false use_xrslam:=true \
     xrslam_image_topic:=/ACTUAL/rectified/image \
     xrslam_camera_info_topic:=/ACTUAL/camera_info \
     xrslam_slam_yaml:=/absolute/path/to/leroy_slam.yaml \
     xrslam_device_template_yaml:=/absolute/path/to/calibrated_sensor.yaml \
     xrslam_camera_imu_offset_sec:=MEASURED_SECONDS
   ```

   This initially keeps `xrslam_pose_convention_validated:=false`, so
   odometry output is **blocked**. After nonidentity pose/TF convention tests
   and a supervised safety review, explicitly override it to `true` for
   further non-actuating VIO tests. XRSLAM must wait for TF2 and CameraInfo
   rather than a fixed sleep.
5. **Odometry ownership:** Confirm
   `ros2 topic info --verbose /odom` reports exactly one publisher (XRSLAM);
   `ros2 run tf2_ros tf2_echo odom base_link` resolves only after tracking
   and gating; `ros2 topic echo /xrslam/tracking_status` reports state.
   Check finite pose, monotonic time, axis conventions, continuity and
   latched inhibit on tracking loss. Wheel encoders must not be needed.
6. **Navigation readiness (separate project):** Verify persistent global
   map alignment, an independent metric occupancy map, calibrated real
   metric depth and a single monitored final `/cmd_vel` publisher. Test
   without actuation first. Do not connect Nav2 motion until a separate
   physical-robot safety review explicitly authorizes raising
   `max_wheel_angular_velocity_rad_s` above `0.0`.

`leroy_msgs/msg/EncoderCounts` remains unchanged. The bridge retains a
500 ms MCU watchdog, 100 ms keepalive, 250 ms command timeout, shutdown stop,
and motor geometry solely for mecanum command conversion.

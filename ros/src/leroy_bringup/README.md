# leroy_bringup

Top-level ROS 2 Jazzy orchestration for Leroy hardware, robot description,
USB camera, rectification and optional DepthART. All existing camera settings
remain configurable; `use_camera` and `use_depthart` still default to true.

```bash
ros2 launch leroy_bringup robot.launch.py use_depthart:=false
```

## Optional XRSLAM

`use_xrslam` defaults to **false**. When enabled it starts Leroy's in-tree `leroy_xrslam` adapter, linked against the separately built, ROS-free [XRSLAM](https://github.com/j0yk1ll/xrslam) library. Its separately built
XRSLAM library and calibrated sensor
YAML files must be available in the *same* sourced runtime/container.

`use_xrslam:=true` requires explicit
`xrslam_slam_yaml`, `xrslam_device_template_yaml`,
`xrslam_image_topic`, and `xrslam_camera_info_topic`.
The SLAM YAML's `sensor.mode` selects the pipeline:

- `monocular`: visual-only diagnostic; `xrslam_camera_imu_offset_sec` can be
  omitted (or set to zero), no IMU subscription or IMU TF lookup; no metric
  odometry, pose output or map TF may be enabled because scale is unknown.
- `monocular_imu`: visual-inertial; a measured finite
  `xrslam_camera_imu_offset_sec` is required and the IMU topic defaults to
  `/imu/data_raw`.

The camera optical frame follows `camera_frame`; static transforms come
from the URDF via `robot_state_publisher`. Both modes retain the
`xrslam_pose_convention_validated:=false` default and disabled map TF.

The adapter stays gated with
`xrslam_pose_convention_validated:=false` by default and no `map -> odom`
publication. No launch sleep or guessed calibration values are injected.
`use_camera:=false` permits externally provided camera images for replay.

See `ros/XRSLAM_INTEGRATION.md` for setup and staged acceptance checks.

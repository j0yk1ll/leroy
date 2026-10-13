# leroy_bringup

Top-level ROS 2 Jazzy orchestration for Leroy hardware, robot description,
USB camera, rectification and optional DepthART. All existing camera settings
remain configurable; `use_camera` and `use_depthart` still default to true.

```bash
ros2 launch leroy_bringup robot.launch.py use_depthart:=false
```

## Optional XRSLAM

`use_xrslam` defaults to **false**. When enabled it starts Leroy's in-tree `xrslam_ros2` adapter, linked against the separately built, ROS-free [XRSLAM](https://github.com/j0yk1ll/xrslam) library. Its separately built
XRSLAM library and calibrated sensor
YAML files must be available in the *same* sourced runtime/container.

`use_xrslam:=true` requires explicit
`xrslam_slam_yaml`, `xrslam_device_template_yaml`,
`xrslam_image_topic`, `xrslam_camera_info_topic`, and a **measured**
`xrslam_camera_imu_offset_sec`. The IMU topic defaults to
`/imu/data_raw`; the camera optical frame follows `camera_frame`.
Static transforms come from the URDF via `robot_state_publisher`.

The adapter stays gated with
`xrslam_pose_convention_validated:=false` by default and no `map -> odom`
publication. No launch sleep or guessed calibration values are injected.
`use_camera:=false` permits externally provided camera images for replay.

See `ros/XRSLAM_INTEGRATION.md` for setup and staged acceptance checks.

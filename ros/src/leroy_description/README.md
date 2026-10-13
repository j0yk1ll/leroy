# leroy_description

Canonical fixed sensor geometry for Leroy, published once via
`robot_state_publisher`. The bridge and XRSLAM must not publish duplicate
static sensor transforms. The TF tree is:

```text
base_link
├── imu_link
├── front_camera_link
│   └── front_camera_optical_frame
│       └── camera_optical_frame
└── workspace_camera_link
    └── workspace_camera_optical_frame
```

**PROVISIONAL** Xacro values: IMU translation
`(0.095, -0.095, 0.0)` m with identity rotation; front camera tilted 30°
down from forward; workspace camera tilted 60° down. Camera translations and
IMU height are currently zero **approximations**, not measured calibrations.
Do not mark camera/IMU VIO as validated with these assumptions.

Measure the IMU and front camera 6-DoF extrinsics on the physical robot, verify
that the IMU uses +X forward, +Y left, +Z up, that the optical frame uses +X
right, +Y down, +Z forward, and that the rectified image header identifies
`camera_optical_frame`. Then update the **single authoritative** Xacro. The
XRSLAM adapter will look up its camera/IMU extrinsics with TF2 rather than
using duplicated pose values in device YAML.

```bash
ros2 launch leroy_description description.launch.py
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo base_link workspace_camera_optical_frame
```

No static `odom -> base_link` or fake identity `map -> odom` belongs here.

# leroy_description

Fixed robot frames for Leroy.

Current tree:

```text
base_link
├── imu_link
├── front_camera_link
│   └── front_camera_optical_frame
│       └── camera_optical_frame
└── workspace_camera_link
    └── workspace_camera_optical_frame
```

Known geometry:
- IMU: +0.095 m forward, -0.095 m left, zero rotation.
- Front camera: 30° downward from forward.
- Workspace camera: 60° downward from forward.
- Camera translations and IMU height are currently approximated as zero.

Launch:

```bash
ros2 launch leroy_description description.launch.py
```

Validate:

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link front_camera_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo base_link workspace_camera_optical_frame
```

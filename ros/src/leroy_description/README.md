# leroy_description

Canonical fixed sensor geometry for Leroy, published once via
`robot_state_publisher`. The bridge and XRSLAM must not publish
duplicate static sensor transforms.

**Edit the hardware values in one file:**
`config/sensor_mounts.yaml`.
`urdf/leroy.urdf.xacro` loads that YAML and provides the frame structure
and fixed ROS optical-axis conversions. `CMakeLists.txt` installs the
configuration along with the launch and URDF files.

The YAML defines three physical mounting transforms, in **metres**
(`xyz`) and **radians** (`rpy`):

- `mounts.imu`: `base_link -> imu_link`
- `mounts.front_camera`: `base_link -> front_camera_link`
- `mounts.workspace_camera`: `base_link -> workspace_camera_link`

Coordinates are +X forward, +Y left, +Z up. The original provisional
values were **preserved exactly**: IMU position (+0.095, −0.095, 0) m
with identity frame rotation; front camera pitch 30° and translation
(0, 0, 0) m; workspace camera pitch 60° and translation (0, 0, 0) m.
The two zero camera translations and IMU height are placeholders,
not measured sensor centers. The `status: provisional` field records
this; it is not a runtime permission flag.

The published IMU accel/gyro messages are already rotated by the
MCU into robot-aligned axes, using the mounting profile
`forward = −ICM X`, `left = −ICM Y`, `up = +ICM Z`.
The **ROS `imu_link` rotation stays identity**; do not rotate the
sensor package axes a second time in this YAML. PR #155 separately
centralizes the firmware's chip-axis profile; it does not change the
physical IMU translation configured here.

The fixed TF tree is:

```text
base_link
├── imu_link
├── front_camera_link
│   └── front_camera_optical_frame
│       └── camera_optical_frame
└── workspace_camera_link
    └── workspace_camera_optical_frame
```

Optical frames follow ROS's +X right, +Y down, +Z forward convention.
Their fixed -90°/-90° conversion remains in Xacro rather than YAML,
because that is a coordinate convention and not a hardware-specific
mounting measurement. The legacy `camera_optical_frame` alias
retains identity translation/rotation.

To inspect the configured robot frame tree on a **reviewed isolated ROS
system only**, the normal launch remains:

```bash
ros2 launch leroy_description description.launch.py
```

To override the default configuration when evaluating Xacro, use the
Xacro argument `sensor_mounts_file:=/absolute/path/to/mounts.yaml`.
Editing YAML or changing its path only affects a ROS description built
or launched with that configuration. **Existing immutable containers
and accepted physical V3 tests do not change automatically.**

For the next *no-device* verification of the source configuration
in the pinned sensor image (from the repository `ros/` directory),
run:

```bash
bash ./m23bv_v3_stage3_sensor_mounts_yaml_no_device.sh
```

This check sources no hardware, runs Xacro and validates the generated
TF origins match the YAML, then changes one value in a **temporary
container-only YAML copy** and proves it changes the generated URDF.
It does not launch ROS nodes or publish TF. Do not enable XRSLAM pose,
map TF, real Nav2, MCU, recording or motor control; camera–IMU
temporal phase and sensor-center extrinsics are still unvalidated.

No static `odom -> base_link` or fake identity `map -> odom`
belongs in this package.

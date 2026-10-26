# leroy_description — YAML-configured sensor TF

## Current validation result

**Operator PASS:** `STAGE3_SENSOR_TF=PASS_YAML_STATIC_TF_NO_DEVICE`.
In the pinned, network-isolated container without devices, the
YAML was validated against all six previous static TF edges,
`tf2_ros` action objects were successfully created (but not
started), a temporary 1-mm YAML edit altered the publisher arguments,
and invalid settings failed closed. No full image rebuild occurred.
There has **not** yet been a real `/tf_static` publisher runtime
test. The active physical hardware remains untouched.


This package **does not use URDF or Xacro**. A small ROS 2 launch file
reads one YAML and starts only six standard
`tf2_ros/static_transform_publisher` processes to publish fixed
sensor transforms on `/tf_static`.

## Edit sensor mounting coordinates here

`ros/src/leroy_description/config/sensor_mounts.yaml`

- `mounts.imu`: `base_link -> imu_link`
- `mounts.front_camera`: `base_link -> front_camera_link`
- `mounts.workspace_camera`: `base_link -> workspace_camera_link`
- `fixed_frames.front_camera_optical`:
  `front_camera_link -> front_camera_optical_frame`
- `fixed_frames.camera_optical_alias`:
  `front_camera_optical_frame -> camera_optical_frame`
- `fixed_frames.workspace_camera_optical`:
  `workspace_camera_link -> workspace_camera_optical_frame`

YAML units: `xyz` **metres**; `rpy` **radians**, in roll/pitch/yaw
order. `base_link` is +X forward, +Y left, +Z up.
Optical frames follow ROS +X right, +Y down, +Z forward. All six
transform values, including the optical-frame conventions, are visible
in the YAML; frame IDs and topology are fixed by the launcher to avoid
accidental map/odom authority or duplicate children.

The values were migrated **without numeric changes** from the previous
Xacro: IMU (+95, −95, 0) mm with identity robot-frame orientation,
front camera provisional +30° pitch and zero translation, workspace
camera provisional +60° pitch and zero translation. This is a source
configuration migration, **not calibration acceptance**. The YAML
currently records `status: provisional` and the launch refuses other
status labels rather than pretending a setting confers permission.

`/imu/data_raw` is already robot-aligned by MCU firmware
(forward=−ICM X, left=−ICM Y, up=+ICM Z). **Keep
`mounts.imu.rpy=[0,0,0]`**; the loader rejects a second sensor-package
rotation. Changing a physical IMU chip mounting requires reviewing the
separate firmware profile (draft PR #155), not applying duplicate
rotations in ROS.

## Launch and ownership

The existing launch entry point is unchanged:

```bash
ros2 launch leroy_description description.launch.py
```

An optional configuration override uses:

```bash
ros2 launch leroy_description description.launch.py \
  sensor_mounts_file:=/absolute/path/to/sensor_mounts.yaml
```

The launch is included by Leroy bringup and by the Stage 3 XRSLAM
diagnostic. **Start exactly one publisher per child frame** and never
run the earlier `robot_state_publisher`/URDF TF owner simultaneously;
the two mechanisms would duplicate `/tf_static` authority.
No `odom -> base_link` or `map -> odom` is published here.

The source-only migration does NOT update already-built immutable
Podman sensor or XRSLAM images. Actual runtime adoption requires a
separately reviewed package build/install path. Do not alter the
accepted Stage 3 physical runner, flash the MCU, or enable pose,
Nav2 goals, recording or motor output as part of this conversion.

To verify the new source without accessing hardware, from either
directory run:

```bash
# Repository root:
bash ros/m23bv_v3_stage3_sensor_tf_yaml_no_device.sh

# Alternatively inside ros/:
bash ./m23bv_v3_stage3_sensor_tf_yaml_no_device.sh
```

This isolated test uses the pinned existing ROS Jazzy image to parse
YAML, import the launch without starting nodes, and compare the six
configured TF edges against their old values. It also validates a
temporary YAML override and deliberate invalid configurations.
A PASS does not establish physical sensor centers, camera–IMU phase,
XRSLAM tracking or Nav2 readiness.

# Leroy robot description and sensor TF

The **legacy** `description.launch.py` on main remains unchanged and
uses Xacro with `robot_state_publisher`; the usual robot bringup still
refers to this entry point.

For the separately guarded real-camera Stage 3 integration, use
`live_sensor_mounts.launch.py`. It reads
`config/sensor_mounts.yaml` and publishes exactly six fixed sensor TF
edges with `tf2_ros/static_transform_publisher`. The source YAML is
**provisional** (not physical calibration). The live costmap launch
supplies the regular-file copy at `/etc/leroy/sensor_mounts.yaml`, to
avoid a symlink from colcon's `--symlink-install` mode.

These two launchers are **alternative TF owners** for overlapping sensor
frames; do not start both in one ROS graph. The isolated Stage 3 script
starts only the live sensor TF launcher. It does not start the
motor-capable bridge, dynamic odom TF, map alignment, planners or
controllers.

Frame topology in the live launcher is fixed:

- `base_link -> imu_link`
- `base_link -> front_camera_link`
- `front_camera_link -> front_camera_optical_frame`
- `front_camera_optical_frame -> camera_optical_frame`
- `base_link -> workspace_camera_link`
- `workspace_camera_link -> workspace_camera_optical_frame`

Coordinates are metres and RPY radians. Camera centers and pitch
are placeholders; `/imu/data_raw` is already robot-axis aligned by
the MCU, so `imu_link` remains identity rotation. No `odom` or `map`
edges are created. For the accepted sensor-only runtime and tests, see
`ros/LIVE_STAGE3_REPRODUCIBLE.md`.

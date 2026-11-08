# ROS-owned sensor calibration for XRSLAM (B2)

For **camera-only** `monocular` and `rgbd` modes, the XRSLAM ROS adapter
does not require a sensor device template. The single source of sensor
geometry is ROS:

- Rectified `sensor_msgs/CameraInfo` (configured `camera_info_topic`):
  image width and height, intrinsic projection `P`, pinhole/zero-distortion
  rectified model.
- Static `tf2` transform `base_link <- camera_optical_frame`:
  camera orientation and translation in Leroy's base frame.
- Sensor mode, tracker settings and estimator thresholds remain in
  `slam_yaml`. The frontend keypoint noise variance (in pixel²) is a
  separate estimator parameter, `keypoint_noise_variance_px2` (default 0.5).

XRSLAM's native API currently takes a YAML filename at `XRSLAMCreate`.
The adapter **serializes a temporary YAML file from the ROS messages and
static TF at initialization**. This is an implementation detail, not
a second calibration source or a file the operator maintains.

The adapter rejects an observed change in effective CameraInfo projection
or resolution after initialization, rather than continuing with divergent
sensor geometry. Restart XRSLAM after changing camera calibration or TF;
the native estimator does not support in-place calibration updates.

## Camera-only startup

Ensure the camera/rectifier and robot description TF publisher are running,
then run the normal XRSLAM command **without** `device_template_yaml`:

```bash
ros2 run leroy_localization leroy_xrslam_node --ros-args \
  -p slam_yaml:=/opt/xrslam/configs/rgbd_slam.yaml \
  -p image_topic:=/camera/image_rect \
  -p camera_info_topic:=/camera/camera_info \
  -p camera_optical_frame:=camera_optical_frame \
  -p depth_topic:=/video_depth_anything/depth \
  -p async_depth_stage_a1:=true \
  -p experimental_metric_preview:=true \
  -p pose_convention_validated:=false \
  -p enable_map_tf:=false
```

The old `device_template_yaml` argument is **optional and unused for
camera-only modes**. Avoid passing EuRoC paths to new camera-only launches.

## Remaining IMU compatibility case

The IMU-enabled mode still requires a device template **solely to populate
IMU covariance/noise parameters** not provided by `sensor_msgs/Imu`.
Its camera calibration is overwritten from ROS CameraInfo + TF, and its
IMU geometry is overwritten from `base_link <- imu_link`. Future work
should expose noise parameters through ROS configuration as well; do not
invent them or silently fall back to EuRoC calibration.

**This change does not validate** the XRSLAM body/camera output convention
or gravity alignment. Keep `pose_convention_validated=false`,
`enable_map_tf=false`, `depth_constraints_added=0`, and no motor authority
until those checks are completed.

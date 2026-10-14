# ChArUco-guided camera/IMU time-offset calibration

The existing `utils/charuco/generate.py`, `calibrate.py`, and printed board are reused. The new `utils/charuco/pose.py` imports the existing `load_board` and `detect_charuco` helpers. `calibrate_time_offset.py` estimates only timestamp phase; measured extrinsics in TF and calibrated camera intrinsics remain unchanged. The old `test_cam_sync.py` is a timestamp-spacing diagnostic, **not** physical-event calibration.

## Container setup

From the repository root on the host, with Leroy's camera/bridge container already running:

```bash
podman exec leroy-ros mkdir -p /tmp/leroy_utils/charuco/calibration_board /tmp/leroy_utils/imu
podman cp utils/charuco/calibrate.py leroy-ros:/tmp/leroy_utils/charuco/calibrate.py
podman cp utils/charuco/pose.py leroy-ros:/tmp/leroy_utils/charuco/pose.py
podman cp utils/charuco/calibration_board/board_config.json leroy-ros:/tmp/leroy_utils/charuco/calibration_board/board_config.json
podman cp utils/imu/calibrate_time_offset.py leroy-ros:/tmp/leroy_utils/imu/calibrate_time_offset.py

podman exec -it leroy-ros bash -lc '
 source /opt/ros/jazzy/setup.bash
 source /leroy_ws/install/setup.bash
 python3 /tmp/leroy_utils/imu/calibrate_time_offset.py --self-test
'
```

Requires `cv2.aruco` (OpenCV-contrib), `rclpy`, `tf2_ros`, `rosbag2_py`, and numpy in the running environment; do not change the camera configuration. Verify the physical ChArUco print matches `board_config.json`.

### Manual first (default, sends no motor commands)

```bash
podman exec -it leroy-ros bash -lc '
 source /opt/ros/jazzy/setup.bash
 source /leroy_ws/install/setup.bash
 python3 /tmp/leroy_utils/imu/calibrate_time_offset.py \
  --target /tmp/leroy_utils/charuco/calibration_board/board_config.json \
  --output /tmp/leroy_charuco_time_01 --seconds 75
'
```

Keep the printed board fixed and move the rigid camera/IMU assembly in varied moderate yaw movements. Recording begins after stable board acquisition. The script saves a ROS 2 bag with compressed images, CameraInfo and raw IMU at original sensor timestamps, plus a JSON offset summary (`/tmp/leroy_charuco_time_01_summary.json`). Inspect validation fields: broad/weak or inconsistent correlation **must not** be used for XRSLAM.

### Optional supervised automatic planar motion

**Not a navigation or collision-avoidance system.** The repository currently disables motors through `max_wheel_angular_velocity_rad_s: 0.0`. The script never bypasses that interlock. Only commission motors and independently verify wheel directions, stop behavior, a genuine hardware emergency stop, a physically clear bounded space, and a supervising operator before using `--arm`.

The optional mode publishes short randomized, bounded lateral/longitudinal/yaw `/cmd_vel` movements. It uses target pose to keep the board visible and latches a stop on missing/stale camera or IMU, lost target, range/displacement limit, high yaw rate, or competing motor publisher. ROS bridge and MCU watchdogs remain essential. When those prerequisites are established:

```bash
podman exec -it leroy-ros bash -lc '
 source /opt/ros/jazzy/setup.bash
 source /leroy_ws/install/setup.bash
 python3 /tmp/leroy_utils/imu/calibrate_time_offset.py \
  --target /tmp/leroy_utils/charuco/calibration_board/board_config.json \
  --output /tmp/leroy_charuco_time_02 --seconds 75 \
  --arm --confirm-clear-area --confirm-estop
'
```

The effective estimated offset is defined as **camera header time minus IMU header time for the same physical yaw event**. Do **not** copy it blindly to XRSLAM's `cam0.time_offset`; the downstream sign convention must be independently verified. A camera exposure/readout offset may remain even with strong motion correlation.

This code has offline synthetic checks only. It has not been validated with actual wheel movement on Leroy; `--arm` remains strictly opt-in.

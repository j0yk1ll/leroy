# ChArUco-guided camera/IMU time-offset calibration

The existing `utils/charuco/generate.py`, `calibrate.py`, and printed board are reused. The new `utils/charuco/pose.py` imports the existing `load_board` and `detect_charuco` helpers. `calibrate_time_offset.py` estimates only timestamp phase; measured extrinsics in TF and calibrated camera intrinsics remain unchanged. The old `test_cam_sync.py` is a timestamp-spacing diagnostic, **not** physical-event calibration.

## Existing board and OpenCV compatibility

Use the already printed ChArUco target and its checked-in `board_config.json`. Do not regenerate or reprint the board, change camera intrinsics, or change camera–IMU transforms for this temporal-only calibration. The self-test draws a synthetic board image *in memory* solely to test the detector.

The container's existing OpenCV **4.6.0** supports this board through the legacy `cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length_m, marker_length_m, dictionary)` factory. Its `cv2.aruco.DetectorParameters()` constructor may also produce an invalid native object that segfaults when passed into `detectMarkers`. For legacy detection, this toolkit intentionally calls `cv2.aruco.detectMarkers(gray, dictionary)` **without an explicit parameter object**, matching the successful marker test on Leroy. The self-test now logs marker detection and ChArUco interpolation separately. Do not use OpenCV 4.7+ tuple-constructor syntax with that runtime: it caused native crashes when rendering the board. `calibrate.py` now selects the correct constructor automatically, so there is **no extra OpenCV installation**, no virtualenv, no new image build, and no change to ROS `cv_bridge` or DepthART.

## Container setup

From the repository root on the host, with Leroy's camera/bridge container already running:

```bash
podman exec leroy-ros mkdir -p /tmp/leroy_utils/charuco/calibration_board /tmp/leroy_utils/imu
podman cp utils/charuco/calibrate.py leroy-ros:/tmp/leroy_utils/charuco/calibrate.py
podman cp utils/charuco/pose.py leroy-ros:/tmp/leroy_utils/charuco/pose.py
podman cp utils/charuco/calibration_board/board_config.json leroy-ros:/tmp/leroy_utils/charuco/calibration_board/board_config.json
podman cp utils/imu/calibrate_time_offset.py leroy-ros:/tmp/leroy_utils/imu/calibrate_time_offset.py
podman cp utils/charuco/test_cv_compat.py leroy-ros:/tmp/leroy_utils/charuco/test_cv_compat.py

podman exec -it leroy-ros bash -lc '
 source /opt/ros/jazzy/setup.bash
 source /leroy_ws/install/setup.bash
 PYTHONFAULTHANDLER=1 python3 -u /tmp/leroy_utils/imu/calibrate_time_offset.py --self-test
'
```

Requires the existing OpenCV 4.6.0 ArUco module, `rclpy`, `tf2_ros`, `rosbag2_py`, and NumPy. Verify that the synthetic self-test prints `ChArUco SELF-TEST PASSED` and exits with code `0` before attempting ROS recording. You can then run the compatibility regression tests:

```bash
podman exec -it leroy-ros bash -lc '
 source /opt/ros/jazzy/setup.bash
 source /leroy_ws/install/setup.bash
 PYTHONFAULTHANDLER=1 python3 -u -m unittest discover \
   -s /tmp/leroy_utils/charuco -p test_cv_compat.py -v
'
```

These tests also exercise `interpolateCornersCharuco` on the synthetic image; successful marker detection alone is not sufficient. Do not change the camera configuration or printed board.

### Jazzy bag writer smoke test

Leroy's ROS 2 Jazzy `rosbag2_py.TopicMetadata` Python constructor
requires `id=0` even though rosbag2 assigns topic IDs to its SQLite
storage internally. An older utility revision omitted `id` and failed
immediately after successfully acquiring the printed ChArUco board.
The fixed utility passes `id=0` when registering each topic.

Before repeating a live recording, smoke-test the exact installed bag API.
The test writes and reads three synthetic messages in an automatically
deleted temporary SQLite bag. It does **not** subscribe to sensors or move
the robot.

```bash
podman cp utils/imu/test_rosbag_writer.py leroy-ros:/tmp/leroy_utils/imu/test_rosbag_writer.py
podman exec -it leroy-ros bash -lc '
 source /opt/ros/jazzy/setup.bash
 source /leroy_ws/install/setup.bash
 python3 -m unittest discover -s /tmp/leroy_utils/imu -p test_rosbag_writer.py -v
'
```

Use a new `--output` for each run: rosbag creates the output directory
as soon as acquisition succeeds, even if topic registration later fails.
For example, after `/tmp/leroy_charuco_manual_02`, use
`/tmp/leroy_charuco_manual_03`. Never delete a bag unless you've
confirmed it contains no useful data.

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

**Not a navigation or collision-avoidance system.** The bridge code defaults to disabled motor output (`wheel_velocity_to_pwm_gain: 0.0`); the checked-in robot YAML opts into a **provisional supervised low-duty trial**, using `wheel_velocity_to_pwm_gain: 12.75` PWM/(wheel rad/s) with an independent `max_pwm: 64` (25% of full PWM). The gain is NOT a physical speed limit: increasing it increases PWM for a given command. The automatic script refuses to arm unless this gain is positive, `max_pwm <= 64`, `cmd_vel_timeout_s <= 0.30`, and `1 <= watchdog_ms <= 500`. Cached motor gains, geometry and safety parameters are read-only after bridge startup so ROS parameter queries match the effective settings. These checks do not replace independently verified wheel directions, stop behavior, a physical emergency stop, a cleared bounded area, and a supervising operator. Source changes require rebuilding/restarting the bridge; neither editing YAML nor calling `ros2 param set` changes the running bridge's motor settings.

The optional mode publishes short randomized, bounded lateral/longitudinal/yaw `/cmd_vel` movements. It uses target pose to keep the board visible and latches a stop on stale camera or IMU, sustained tracking loss (>0.30 s), range/displacement violation, high yaw rate, or competing motor publisher. A single untracked detector frame immediately commands **zero motion**, and movement only resumes after 0.20 s of stable detections; it is not misclassified as a geometry violation. Genuine boundary failures now print which limit was exceeded. ROS bridge and MCU watchdogs remain essential. When those prerequisites are established:

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

#### Diagnose low temporal correlation from an existing bag (no robot movement)

A complete supervised yaw-only run can still produce a weak alignment score
when adjacent-frame ChArUco PnP orientation jitter dominates real yaw.
For example, one 20-second field trial had IMU yaw std 0.0807 rad/s,
visual yaw std 0.1955 rad/s, and correlation only 0.264
(required >= 0.65). **Neither sufficient IMU excitation nor agreement
between weak half-run offsets makes that estimate valid.**

Before any longer motor run, analyze the EXISTING bag with the read-only
`analyze_charuco_bag.py` utility. It uses the same physical printed board,
the CameraInfo recorded in the bag, and the **existing live TF** from
`camera_optical_frame` to `imu_link` (it does not change extrinsics).
It replays detections, rejects implausible pose flips, and compares current
adjacent-frame visual rotations with longer 120, 250 and 400 ms intervals,
each integrated against the actual IMU gyro timestamps:

```bash
podman exec -it leroy-ros bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  python3 /tmp/leroy_utils/imu/analyze_charuco_bag.py \
    --bag /tmp/leroy_charuco_auto_yaw_02 \
    --target /tmp/leroy_utils/charuco/calibration_board/board_config.json
'
```

This does **not** publish motor commands, alter recorded bags or change the
live calibration script's default algorithm. Longer relative-rotation
intervals reduce orientation-derivative jitter but smooth real dynamics,
correlate successive samples, and can broaden the timing peak. Treat all
reported lag candidates as **diagnostic only**. Only adopt a revised
estimator after independent synthetic/field validation and a sufficiently
long recording with strong correlation, a distinct peak, and consistent
split estimates. Do not propagate any `lag_ms` to XRSLAM yet.

#### Orientation-domain timing diagnosis (read-only)

In the completed 20-second yaw-only recording,
`leroy_charuco_auto_yaw_02`, adjacent-frame visual yaw had only
0.264 correlation with the gyro. A 250 ms visual rotation baseline
improved correlation to 0.733, but its peak separation was only 0.017
(required >=0.06 in the established online estimator). A 400 ms
baseline improved correlation to 0.801 but generated a broad,
split-inconsistent peak. **Do not interpret these values as a valid
camera/IMU time offset.**

For an independent diagnostic that avoids dividing noisy PnP angle
differences by ~30–70 ms frame spacings, the read-only
`analyze_charuco_orientation.py` uses the existing saved bag and
existing live camera-to-IMU TF. It compares the visual **relative yaw
angles** against the trapezoidally integrated measured IMU yaw rate
over candidate timing offsets. For each candidate it fits only an
unknown starting angle and linear gyro drift, without changing TF,
intrinsics, bag timestamps or the online estimator. It compares
full-recording lag, per-half lag, residual angular RMSE, and the
relative increase in residual loss when moving at least 40 ms away
from the optimum. A broad flat residual-loss valley means the
data still cannot uniquely identify an offset.

Copy `utils/imu/orientation_timing.py` and
`utils/imu/analyze_charuco_orientation.py` into the running container's
`/tmp/leroy_utils/imu/` directory, then run:

```bash
podman exec -it leroy-ros bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  python3 /tmp/leroy_utils/imu/analyze_charuco_orientation.py \
    --bag /tmp/leroy_charuco_auto_yaw_02 \
    --target /tmp/leroy_utils/charuco/calibration_board/board_config.json
'
```

This analysis does **not** publish to `/cmd_vel` and contains no
motor-controller logic. The approach assumes a stationary board
and limited yaw excursions (so camera relative rotation vectors
have no full-turn phase ambiguity). It uses ordinary least squares
and is vulnerable to residual PnP outliers; reported values are
strictly **diagnostic**. Run the ROS-free synthetic tests via:

```bash
python3 -m unittest discover -s utils/imu -p test_orientation_timing.py -v
```

No lag reported by this utility should be applied to XRSLAM without
a separate validated estimator and independent repeatable recording.

#### Planar-PnP pose-flip rejection

A real yaw-only trial produced a one-frame ChArUco pose jump: 0.425 m
translation and 21.9 degrees rotation in 36 ms, immediately returning to
the original pose on the next frame. Board center remained near (0.495,
0.486) at 1.18 m. The low 0.18–0.23 px reprojection error did **not**
distinguish the incorrect planar-PnP branch from a true camera pose.

To avoid treating one implausible PnP branch flip as actual robot travel,
the temporal calibration script now withholds a pose that jumps farther
or rotates faster than a deliberately generous frame-to-frame continuity
envelope. On rejection it **commands zero motor velocity immediately**,
clears the good-tracking timer and waits for 0.20 s of stable reacquisition.
A sustained absence of accepted poses still triggers the hard 0.30 s
tracking-loss stop. Real image-edge/range violations stop immediately,
and plausible physical movement beyond 0.18 m from the starting pose
continues to trigger the unchanged displacement stop. Even a true but
physically implausible displacement forces zero output and, if persistent,
a hard tracking-loss stop. No safety limits are increased.

`HOLD: ChArUco PnP pose jump...` messages and the
`counts.rejected_pose_jumps` summary field identify frames excluded from
motion control and from visual angular-velocity estimation. The filter is
not a new camera-to-IMU spatial calibration. It does **not** guarantee
that remaining poses are accurate; a valid temporal offset still requires
signal/correlation/split consistency. Run the ROS-free tests via:

```bash
python3 -m unittest discover -s utils/imu -p test_charuco_motion_safety.py -v
```

#### Yaw-focused excitation when the IMU signal is too weak

If a full supervised recording completes but the offset estimator reports
`Insufficient varying yaw signal`, first inspect the **IMU** yaw variation,
not only the noisier frame-to-frame visual estimate. The estimator reports
`imu_yaw_std_rad_s`, `visual_yaw_std_rad_s`, and
`imu_peak_abs_yaw_rad_s`; each signal must have a standard deviation of
at least 0.035 rad/s for reliable correlation to be considered.

The optional `--yaw-only` mode (requires `--arm` and both safety
acknowledgments) alternates bounded yaw-left and yaw-right commands
at nominal +/-0.21 rad/s, rather than randomly mixing translation with yaw.
For the configured wheel geometry and gain, that requests approximately
+/-61 PWM before the independent +/-64 PWM cap. Actual wheel movement may
still be insufficient under load. Tracking, center/range/displacement guards,
gyro cutoff, startup requirements, timeout, and hardware E-stop requirements
are **unchanged**. This flag deliberately does **not** increase `max_pwm`.

First run **20 seconds**, in the same cleared supervised area as before,
using a fresh output directory. Only after observing actual stable rotation
and safe stops should you attempt a 75-second capture:

```bash
podman exec -it leroy-ros bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  python3 -u /tmp/leroy_utils/imu/calibrate_time_offset.py \
    --target /tmp/leroy_utils/charuco/calibration_board/board_config.json \
    --output /tmp/leroy_charuco_yaw_01 --seconds 20 --wait 120 \
    --arm --yaw-only --confirm-clear-area --confirm-estop
'
```

Do not lower the excitation/offset-quality thresholds to force a valid
result; noisy visual yaw can vary strongly even when the robot barely turns.

The effective estimated offset is defined as **camera header time minus IMU header time for the same physical yaw event**. Do **not** copy it blindly to XRSLAM's `cam0.time_offset`; the downstream sign convention must be independently verified. A camera exposure/readout offset may remain even with strong motion correlation.

This code has offline synthetic checks only. It has not been validated with actual wheel movement on Leroy; `--arm` remains strictly opt-in.


#### Independent raw ChArUco corner bearing comparison (offline)

The original 20-second, completed 75-second, and interrupted ~38-second
recordings show **estimator-dependent** lag estimates from the *same*
reconstructed PnP poses. Adjacent-frame yaw yields approximately -42,
-58 and -66 ms, while comparing PnP orientation against integrated gyro
yields -52, -90 and -114 ms respectively. Requiring more detected corners
shifts the long-recording orientation estimates towards zero; allowing the
integrated gyro angle a free scale changes these lag results by at most
2 ms. **These are not validated sensor offsets.**

`analyze_charuco_bearings.py` provides a deliberately independent,
read-only observation. For each identified ChArUco chessboard corner
it uses **raw detected pixel positions**, the calibrated CameraInfo
intrinsics/distortion, and the **existing** camera-to-IMU TF to calculate
the camera ray's azimuth in IMU XY. It does **not use solvePnP**
or calculate board orientation. A planar homography is used **only for
rejecting inconsistent corner IDs**, never as a pose measurement.

Each well-observed corner is fitted separately against the integrated
raw IMU yaw trace over -250..+250 ms, with nuisance intercept, time
drift and angular gain. Each candidate lag uses an identical subset of
frames. Results include per-corner full/half lags, gain, bearing residual
RMS, and timing-cost sharpness:

```bash
python3 -m unittest discover -s utils/imu -p test_corner_bearing_timing.py -v

python3 utils/imu/analyze_charuco_bearings.py \
  --bag /tmp/leroy_charuco_auto_yaw_75s_03 \
  --target utils/charuco/calibration_board/board_config.json
```

In Leroy's running ROS container, source both ROS Jazzy and
`/leroy_ws/install/setup.bash` first and use the container's existing
`/tmp/leroy_utils/imu/` copies of the scripts.

A physical fixed corner observed from an off-center camera undergoing
yaw can exhibit parallax; fitted bearing gain is diagnostic and cannot
be interpreted as a calibration correction. Correlated common-mode
pixel timing errors are not ruled out merely by consistent corner
results. The tests use synthetic IMU and pixel-bearing trajectories;
the exact recordings have **not** yet been analyzed with this method.
This utility creates **no** motor publisher, does not write bags, and
does not change intrinsics, extrinsics, controller limits or any
XRSLAM parameter. All output is **diagnostic only**, with no
`valid:true` designation.

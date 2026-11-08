# C1 isolated XRSLAM trajectory consistency test

Use the corrected-timestamp `leroy_cam_imu_05` bag, the existing static
TF and a provisional bias-hypothesis configuration. Do not enable navigation,
motor authority, `/odom`, map TF, pose convention validation or metric preview.

## Inputs and interpretation

The offline bias replay harness now records these topics in its `status_bag`:

- `/xrslam/tracking_status` (state transitions)
- `/xrslam/localization_valid` (expected false)
- `/xrslam/visual_pose` (XRSLAM unscaled, isolated pose)
- `/imu/data_raw` (the same replayed IMU samples)

`xrslam_trajectory_diagnostics` reads only this MCAP bag. It reports
pose count, timestamp continuity, relative path/steps and vertical
excursion in **unverified visual units**, and body-relative incremental
quaternion rotations versus integrated IMU angular rate over matching
timestamps. The comparison assumes the provisional Leroy static TF's
`imu_link` axes are aligned with `base_link`; it prints direct and
opposite-sign yaw comparisons because the XRSLAM pose convention is
not yet independently verified. A correlation alone cannot certify it.

It does **not** treat a small start/end difference as loop closure without
externally verified start/end positions, nor estimate metric scale.

## Host deployment

```bash
cd ~/Code/leroy
git switch feature/nav2-functional-foundation
git pull --ff-only origin feature/nav2-functional-foundation
git archive HEAD ros/src/leroy_localization | \
  podman exec -i leroy-viz tar -xf - --strip-components=2 -C /leroy_ws/src
podman cp ros/scripts/xrslam_bias_replay_offline.sh \
  leroy-viz:/tmp/xrslam_bias_replay_offline.sh
```

In `leroy-viz`:

```bash
cd /leroy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select leroy_localization --cmake-args \
  -DXRSLAM_ROOT=/opt/xrslam \
  -DXRSLAM_LIBRARY=/opt/xrslam/lib/libxrslam.so
source /leroy_ws/install/setup.bash
```

Verify domain 191 has no active XRSLAM or IMU/camera publishers:

```bash
ROS_DOMAIN_ID=191 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros2 node list --no-daemon
```

Old static TF publishers from the first harness revision may still exist.
Do not kill normal bringup TF publishers; check PPID and process ownership
before stopping orphaned experiment processes.

## One new replay and analysis

Choose **one** exploratory bias scenario initially; no new bias sweep is
necessary. This example uses the mid hypothesis, not a calibrated value.
A new output root avoids modifying earlier evidence:

```bash
bash /tmp/xrslam_bias_replay_offline.sh mid \
  /leroy_ws/calibration/leroy_cam_imu_05 \
  /leroy_ws/calibration/xrslam_trajectory_v1
ros2 bag info /leroy_ws/calibration/xrslam_trajectory_v1/mid/status_bag

ros2 run leroy_localization xrslam_trajectory_diagnostics \
  /leroy_ws/calibration/xrslam_trajectory_v1/mid/status_bag \
  > /leroy_ws/calibration/xrslam_trajectory_v1/mid/trajectory_summary.json
cat /leroy_ws/calibration/xrslam_trajectory_v1/mid/trajectory_summary.json
```

If recorder reports no `/xrslam/visual_pose` or no `/imu/data_raw`,
do not interpret the trajectory analysis. Diagnose subscription and
playback before trying alternative bias parameters.

## Limits

The historical bag has no independent ground-truth trajectory, and
manual motion may include translation and parallax. Pose frames are
`xrslam_visual_world` and cannot be promoted to `odom` or `map`.
A successful tracking status is not proof of physically meaningful scale.
A frame-convention check, controlled measured-displacement test and
tracking continuity assessment remain commissioning prerequisites.

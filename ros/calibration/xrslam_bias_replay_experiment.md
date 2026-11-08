# C1 offline XRSLAM IMU bias sensitivity replay

**Experimental only. No model calibration or pose authority is established.**

This runs the existing `leroy_cam_imu_05` camera + IMU recording
against the existing *unaltered* static TF and the C1
`xrslam_visual_inertial.launch.py` launch. It starts no real camera,
motor bridge, VDA, Nav2, or active odometry authority.

Measured white-noise covariance density estimates (Allan-derived, isotropic):
- gyro: `6.850035236e-8`
- accel: `1.084722832e-5`

The bias values below are *hypotheses for sensitivity testing only*
(not fitted from the 33-minute stationary bag):

| Scenario | `imu_cov_bg_diagonal` | `imu_cov_ba_diagonal` |
| --- | ---: | ---: |
| low | 1e-12 | 1e-10 |
| mid | 1e-10 | 1e-8 |
| high | 1e-8 | 1e-6 |

Parameter values are the continuous-time covariance coefficients used
by the native XRSLAM preintegrator. Each scenario varies both bias
noise covariances by a factor of 100 relative to the preceding case;
the values are not safe defaults, and relative sensitivity is the only
purpose of this sweep. `camera_imu_offset_sec:=0.0` is a *provisional*
offline timing hypothesis based on corrected camera timestamps and
two temporal diagnostics; it is not a certified zero-time calibration.

## Deployment

On host:

```bash
cd ~/Code/leroy
git switch feature/nav2-functional-foundation
git pull --ff-only origin feature/nav2-functional-foundation
podman cp ros/scripts/xrslam_bias_replay_offline.sh leroy-viz:/tmp/xrslam_bias_replay_offline.sh
```

Inside `leroy-viz`, **don't source your running robot graph into this
experiment**: the script sets ROS_DOMAIN_ID=191 and UDPv4 for its child
processes, uses the installed Leroy stack and fixed TF launch, and
refuses to join a domain with existing camera/IMU/XRSLAM/odom topics.

```bash
bash /tmp/xrslam_bias_replay_offline.sh low
bash /tmp/xrslam_bias_replay_offline.sh mid
bash /tmp/xrslam_bias_replay_offline.sh high
```

Run each sequentially and inspect logs between runs. Results appear
under `/leroy_ws/calibration/xrslam_bias_sensitivity/{low,mid,high}/`:
`inputs.json`, `tf.log`, `xrslam.log`, `playback.log`,
`status_record.log`, and `status_bag`.

The script sets `--rate 0.25` to reduce playback delivery pressure
on the CPU-only laptop. It preserves the original ROS message header
timestamps; there is no camera acquisition or live input.
`use_sim_time` remains false, and the native estimator consumes
header timestamps.

**Never run the script twice simultaneously**, and do not run another
node in domain 191. If that domain is occupied, the experiment should
fail closed rather than be run alongside unknown nodes.

## Interpretation

Check whether XRSLAM reports an initialized/tracking state, whether
native tracked features appear and whether results remain stable across
assumptions. Inspect whether any `imu_gap`, `camera_queue_overflow`,
`configuration_error`, `waiting_calibration`, `initializing`,
or `tracking_fail` events occur.

These experiments do NOT certify 3D pose, metric scale or covariance.
Only record status events and logs. A failed planar-motion initialization
can be due to insufficient translational excitation, rather than noise
covariance. No `/odom`, `map->odom`, metric preview, motor access,
or navigation authority is authorized by these tests.

When interpreting results, first verify that the native runtime .so
corresponds to the inspected XRSLAM sources and that the camera
timestamp fix remains in any future deployment. Tests cannot identify
true bias random walks merely by sweeping hypotheses.

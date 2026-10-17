# M21 — Offline temporal evidence readiness (NO hardware access)

M20 passed its real 120-second concurrent camera/IMU ROS message contract: 6001
IMU messages at 50 Hz; 119.886 seconds receipt overlap; host-relative MCU
clock estimate -41.08 ppm; START/STOP ACKs PASS; capture exit 0.
The tested host camera was /dev/video8, mapped inside M20 as /dev/video2.

## Scope

This offline script parses the existing m20_real_camera_imu.log without ROS,
containers, serial devices, cameras, or robot hardware. It independently checks
the embedded M16 passive observer report for the camera, CameraInfo, and IMU
counts, provenance, receipt freshness, header-age and receipt-gap metrics.

The READY_FOR_M21_CORRELATED_TIMING_EXPERIMENT_DESIGN verdict represents only
evidence readiness. It does not validate exposure-to-IMU acquisition offset,
the unknown MCU-to-host transport delay, or camera-IMU extrinsics.
Do not subtract aggregate camera/IMU ROS header ages to claim physical offset.

## Read-only host invocation

From the Arch Linux repository checkout:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21-offline-camera-imu-timing-readiness
  git pull --ff-only origin feature/m21-offline-camera-imu-timing-readiness
  cd ros
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s . -p m21_offline_timing_test.py -v
  PYTHONDONTWRITEBYTECODE=1 python3 m21_offline_timing_audit.py m20_real_camera_imu.log
) 2>&1 | tee m21_offline_timing_audit.log
M21_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M21_OFFLINE_EXIT_CODE=$M21_OFFLINE_EXIT"
```

The program fails closed if its M16 passive JSON is missing. The previously
pasted M20 summary alone does not contain individual camera stream counts.

## Not yet implemented: physical correlated-event calibration

A later experiment must collect synchronized evidence of one physical event
observed in camera pixels AND inertial motion, with justified uncertainty and
an independently checked event correspondence. It requires a new safety
review and physical authorization before any MCU access. Do not start motors,
the production leroy-ros container, XRSLAM odometry, Nav2, or DepthART.
DepthART CPU performance work remains deferred.

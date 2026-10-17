# M21I — Synthetic 120-second-scale camera/IMU metadata workload

## Motivation

M21H fake-serial fault tests passed 5/5 and M21H_SIM_EXIT_CODE=0.
PR #73 verified merged at 7daab2690446c2205daa8f579746c18c0ffc48a4.
M20's real aggregate audit previously measured accepted 2082 rectified
camera images, 3578 CameraInfo messages and 6001 IMU packets over 120s.
A 1.351-second maximum camera receipt gap needs explicit attention.

M21I uses only the M21D pure-Python metadata recorder. It synthesizes
the same COUNTS (total 11661) and an approximately M20-shaped timing
pattern. EVERY individual event is invented by the fixture and is
unconditionally labeled SYNTHETIC. This is NOT a replay of real per-frame
data and cannot recover any real acquisition offset or camera/IMU
extrinsics. No ROS, Podman, serial, V4L2 or motor runtime is accessed.

## Offline data generated

- Rectified images: 2082 local callback indices, frame camera_optical_frame,
  and one intentionally inserted 1.351-second receive gap
- CameraInfo: 3578 events over 120 seconds
- IMU: 6001 source events at 20ms spacing, fully sequential MCU seq and
  uint32 MCU t_us crossing one wrap boundary, with SYNTHETIC gyro values
- Camera first receipt 0.050s, last 119.936s, giving a 119.886s
  intersection with the CameraInfo and IMU receipt windows
- Deliberately different simulated ROS header age for each topic; this
  does NOT quantify physical camera exposure-to-IMU sampling phase
- Thread-safe M21D buffer limit 12000 events; output bounded to <=8MiB
  and no raw image pixels or device payloads stored

Seven pure-Python tests verify actual output volume, exact gap and overlap
calculation, uint32 wrap without sequence loss, byte/pixel bounds,
uncalibrated synthetic provenance, existing-file no-overwrite, and
capacity exhaustion fail-closed.

## Run hardware-free tests on Arch

Run from the Leroy repository checkout in an interactive Bash terminal:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21i-full-window-metadata-replay
  git pull --ff-only origin feature/m21i-full-window-metadata-replay
  cd ros
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
    -s . -p m21i_full_window_synthetic_replay_test.py -v

  TMP_M21I="$(mktemp)"
  rm -f "$TMP_M21I"
  trap 'rm -f "$TMP_M21I"' EXIT
  PYTHONDONTWRITEBYTECODE=1 python3 \
    m21i_full_window_synthetic_replay.py "$TMP_M21I"
  PYTHONDONTWRITEBYTECODE=1 python3 m21_record_contract.py "$TMP_M21I"
) 2>&1 | tee m21i_full_window_offline.log
M21I_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M21I_OFFLINE_EXIT_CODE=$M21I_OFFLINE_EXIT"
```

Require Ran 7 tests, OK; M21I_FULL_WINDOW_VERDICT=
SYNTHETIC_M20_SCALE_METADATA_CONTRACT; M21C_FORMAT_REPORT_JSON verdict
M21C_OFFLINE_METADATA_FORMAT_VALID; evidence_type SYNTHETIC;
11661 records; image gap 1.351s; overlap 119.886s;
physical_camera_imu_offset_validated=false; and exit 0.

## Physical boundaries — no test authority granted

The original production leroy-ros remains exited. M21I does not start
the M20 physical runner, open camera/serial or run Nav2, XRSLAM, TF,
odom, DepthART or actuators. Full-scale synthetic throughput and
schema validation cannot establish a real-world acquisition phase.

Before ANY future serial use, both WHEEL and MANIPULATOR motor power
must be physically inhibited, hardware E-stop verified, robot
immobilized, current camera (historically /dev/video8) and MCU identities
confirmed with exclusive ownership, and a separate operator-approved
sensor-only disposable Podman runbook used. A lost MCU STOP ACK means
state is unverified and motor power remains physically inhibited.
DepthART CPU optimization is still deferred.

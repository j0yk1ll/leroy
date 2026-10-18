# M23B — Actual XRSLAM Jazzy monocular adapter with SYNTHETIC CDR replay

## Previous acceptance

M23A operator Arch host: 21/21 corrected offline Python PASS and
isolated Jazzy synthetic rosbag2 CDR roundtrip PASS. PRs #94 and
#95 merged into `main` at `ee2e8008...` and `a4505799...`.
No real recorded full-frame sensor bag has been provided. Previous
M20 physical sensor receipts were aggregate ROS transport evidence;
the M21I synthesized metadata file does NOT contain camera pixels.

## Exact goal of M23B

Execute the **actual installed C++ `leroy_xrslam_node` binary**,
not an imitation, against synthetic full-size camera CDR messages
read from a temporary ROS 2 sqlite3 bag. This exercises ROS graph
discovery, CameraInfo, synthetic camera TF, adapter startup,
the camera subscriber, `XRSLAMPushSensorData` and
`XRSLAMRunOneFrame`, without real sensors or motor authority.

This is a first, MONOCULAR-only replay to avoid falsely labelling
an arbitrary camera-IMU time offset as measured. It does NOT test
visual-inertial mode or claim metric scale, tracking accuracy or
valid sensor calibration. The real adapter starts with
`pose_convention_validated=false` and `enable_map_tf=false`.
Although it advertises an `/odom` ROS publisher, it must publish
**zero Odometry messages**. No `/cmd_vel` publisher is permitted.

## Code and synthetic evidence

- `m23b_synthetic_monocular_bag.py`: deterministic 640x480 mono8
  checkerboard-style test images with intentionally fabricated
  texture and motion, 16 CameraInfo and 16 image samples, 125ms
  simulated acquisition interval; exact matched header stamps.
  Writes a bounded temporary sqlite3 rosbag only. Not real pixels.
- `m23b_real_xrslam_monocular_replay_selftest.py`: preflights that
  bag using the real M23A checker in monocular mode. Starts an
  isolated ROS 2 Jazzy node with **only** camera publishers,
  test-only static `base_link` → `camera_optical_frame` identity TF
  and diagnostic subscriptions; launches the exact XRSLAM C++
  executable from `/leroy_ws/install/leroy_xrslam/...` as a bounded
  child process with known parameters. Waits for real ROS subscriber
  discovery and the XRSLAM `initializing` status, then replays
  the bag with actual CDR deserialization and requires the C++
  log `Submitted 1 camera frames to XRSLAM (monocular)`.
  Continually checks no `/cmd_vel` publishers, ZERO `/odom`
  message receipts and no `localization_valid=true`. Terminates
  the child and shuts ROS down even after exceptions.
- Uses upstream XRSLAM's `visual_only_slam.yaml` **test fixture**.
  Makes a TEMPORARY copy of upstream `euroc_sensor.yaml` with
  OpenCV `%YAML:1.0` header removed for yaml-cpp compatibility.
  This example is NEVER substituted for Leroy's actual calibration.
  The identity TF is also purely synthetic, never a verified extrinsic.
- `m23b_monocular_isolation_contract_test.py`: 12 no-ROS static
  and fixture regressions; not a substitute for actual ROS receipts.
- `Dockerfile.m23b-selftest`, `m23b_no_device_run.sh`: locally
  derive from operator-tested M23A image; no downloads, no
  USB/serial/camera mappings, no production robot launch.

## Absolute boundaries

**Opening actual STM32 CDC may reset or energize wheel/manipulator
motors without any wheel command.** DO NOT open `/dev/ttyACM*`,
attach devices, start the normal `leroy-ros` container, run
`robot.launch.py`, create Nav2 actions, send `/cmd_vel`, or
publish real odometry. No physical test approval is conferred.

This test DOES run the XRSLAM library against fabricated images,
but even a successful real-library frame-submission receipt does
NOT validate VIO tracking, metric scale, tracking failure response,
camera-IMU physical offset, lens calibration, rolling shutter,
camera-to-IMU/body extrinsics, sensor provenance, or motor safety.
Test-only identity TF and the EuRoC sample are NOT robot calibration.
Runtime outputs always set all these physical/real-world claims FALSE.

## Phase A — pure Python on Arch, no ROS or devices

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m23b-xrslam-monocular-synthetic-ros-replay
  git pull --ff-only origin feature/m23b-xrslam-monocular-synthetic-ros-replay
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23b_monocular_isolation_contract_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23b_offline_contract.log
M23B_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23B_OFFLINE_EXIT_CODE=$M23B_OFFLINE_EXIT"
```

Require M23B **12/12 PASS**, M23A **21/21 PASS**,
`M23B_MONOCULAR_ISOLATION_CONTRACT=PASS`,
`M23B_REAL_XRSLAM_RUNTIME_VALIDATED=FALSE`,
`M23B_DEVICE_ACCESS=NONE`, `M23B_OFFLINE_EXIT_CODE=0`.

## Phase B — local base-image preflight and disconnected build

Requires the already tested M23A image
`localhost/leroy-sensors:m23a-xrslam-rosbag-preflight`.
Keep `leroy-ros` **exited**; NEVER start it.

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m23a-xrslam-rosbag-preflight
  podman build --network none --no-cache \
    -f Dockerfile.m23b-selftest \
    -t leroy-sensors:m23b-xrslam-monocular-test .
) 2>&1 | tee m23b_no_device_build.log
M23B_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23B_BUILD_EXIT_CODE=$M23B_BUILD_EXIT"
```

Require `M23B_MONOCULAR_ISOLATION_CONTRACT=PASS`,
`M23B_JAZZY_SYNTHETIC_REPLAY_IMPORT=PASS`,
`M23B_BUILD_EXIT_CODE=0` and no unresolved native libraries.

## Phase C — run actual XRSLAM with fake-only ROS bag

After offline checks and build success, do not map devices:

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m23b-synthetic-xrslam-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m23b-xrslam-monocular-test
) 2>&1 | tee m23b_no_device_ros_replay.log
M23B_SIM_EXIT=${PIPESTATUS[0]}
echo "M23B_SIM_EXIT_CODE=$M23B_SIM_EXIT"
```

Expected markers **only if runtime succeeds**:

```text
M23B_SYNTHETIC_MONOCULAR_BAG_PREFLIGHT=PASS
M23B_REAL_XRSLAM_ROS_SUBSCRIPTIONS=PASS
M23B_REAL_XRSLAM_SYNTHETIC_INITIALIZING=PASS
M23B_REAL_XRSLAM_MONOCULAR_FRAME_SUBMITTED=PASS
M23B_ODOM_AND_CMD_OUTPUT_INHIBITED=PASS
M23B_REAL_XRSLAM_SYNTHETIC_REPLAY_SELFTEST=PASS
M23B_VERDICT=REAL_XRSLAM_MONOCULAR_SYNTHETIC_ONLY
M23B_DEVICE_ACCESS=NONE
M23B_SIM_EXIT_CODE=0
```

If any marker is absent or the process crashes, **do not** accept
M23B and do not relax parameters, disable fail-closed assertions,
mount real hardware or publish fake `pose_convention_validated=true`.
Inspect the compact failure and XRSLAM logs; address exactly the
synthetic test or image issue, then repeat the no-device run.

## Later: measured camera+IMU replay

M23C will require a complete recorded full-frame ROS bag and
independently qualified camera/IMU calibration before asserting
real-data tracking. A synthetic VIO test may separately check
transport and failure behavior but must keep odometry inhibited
and label time/extrinsics as provisional. No physical MCU capture
is authorized by the M23B software-only milestone.

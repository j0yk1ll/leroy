# M23A — XRSLAM offline ROS 2 bag input preflight (NO robot/USB)

## Grounded baseline

M16 previously accepted **actual** front-camera plus CPU DepthART
ROS message contracts. M18/M19 validated actual STM32 IMU streaming
and host-relative MCU rate drift, and M20 accepted 120 seconds of
concurrent real camera and IMU receipt; those are historic sensor
transport observations, not new authorization to open STM32 CDC.
M21I's full-window fixture is entirely SYNTHETIC metadata and carries
**no usable image payloads**. The repository contains no confirmed
usable recorded camera/CameraInfo/IMU rosbag for XRSLAM ingestion.
XRSLAM's optional Jazzy adapter compiles, but live tracking, optical
extrinsics, physical camera/IMU time offset, result-frame convention
and recovery behavior remain unverified.

## M23A objective

Produce a deterministic, read-only preparation gate for a future
recorded-data XRSLAM replay. Inspect full **existing** rosbag2 CDR
Image, CameraInfo and Imu messages without launching any ROS node.
Validate the specific data shape and timestamps the checked-in
`leroy_xrslam_node.cpp` expects, and generate machine-readable
fault reasons before any XRSLAM process receives a bag.

**This milestone does NOT start XRSLAM, publish /odom, or replay
ROS topics yet.** A contract PASS says only the inspected bag looks
structurally usable; it does not certify VIO, calibration, tracking,
sensor correctness, camera/IMU physical phase or safety.

## Source components

- `m23a_xrslam_rosbag_preflight.py`: Python standard-library streaming
  contract with lazy ROS imports; `inspect_bag` uses Jazzy
  `rosbag2_py.SequentialReader` + CDR `deserialize_message` to inspect
  a read-only existing rosbag directory with `metadata.yaml`.
  It checks exact expected topic types, camera optical frame, correct
  rectified CameraInfo P/focal lengths/zero stereo baseline, image
  encoding and payload stride/length, exact CameraInfo/image acquisition
  stamp and geometry association (either record order), sequential
  header stamps, finite IMU motion vectors in `imu_link`, camera gaps
  above 250ms, IMU gaps above 300ms, and coarse IMU timestamp coverage.
  No image pixels, identity strings, secrets or environment paths are
  printed in the compact JSON report.
- `m23a_xrslam_rosbag_preflight_test.py`: 21 pure-Python in-memory
  synthetic tests, requiring NO ROS install or physical device.
- `m23a_rosbag_roundtrip_selftest.py`: inside Jazzy only, generates
  a disposable synthetic sqlite3 rosbag with real serialized ROS 2
  Image/CameraInfo/Imu message types; tests the actual bag reader,
  malformed camera frame and missing IMU topic handling.
- `Dockerfile.m23a-selftest` and `m23a_no_device_run.sh`: require a
  **locally existing** `localhost/leroy-ros:xrslam` image and a working
  `rosbag2_py` installation; fail at import/build time if unavailable.
  Do not pull packages or make network connections as a workaround.

### Hard safety and provenance limits

Only synthetic bytes are generated for the roundtrip. Every output
always sets `xrslam_executed=false`,
`xrslam_tracking_validated=false`,
`extrinsics_validated=false`,
`camera_imu_physical_offset_validated=false`,
`provenance_authenticated=false`,
`physical_motion_authorized=false`, `device_access=false`.
Even `--provenance user_declared_recorded` is just a label; the
tool cannot authenticate the recording or its capture conditions.

Opening real STM32 USB CDC may reset the MCU and energize wheels or
the manipulator without any wheel command. No CDC, V4L2, USB, robot
launch, `leroy_bridge`, raw `/cmd_vel`, Nav2 controller, XRSLAM
runtime or physical sensor mappings are permitted in this milestone.
All activity is confined to offline files or an isolated no-device
Podman process; maintain the M22D bridge zero-wheel lockout.

## Arch offline tests — no ROS or Podman necessary

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m23a-xrslam-rosbag-replay-preflight
  git pull --ff-only origin feature/m23a-xrslam-rosbag-replay-preflight
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23a_offline_contract.log
M23A_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23A_OFFLINE_EXIT_CODE=$M23A_OFFLINE_EXIT"
```

Require `Ran 21 tests`, `OK`,
`M23A_XRSLAM_REPLAY_INPUT_CONTRACT=PASS`,
`M23A_VERDICT=OFFLINE_BAG_PREFLIGHT_ONLY`,
`M23A_DEVICE_ACCESS=NONE`, `M23A_OFFLINE_EXIT_CODE=0`.

## Build Jazzy synthetic bag reader image — no network

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-ros:xrslam
  podman build --network none --no-cache \
    -f Dockerfile.m23a-selftest \
    -t leroy-sensors:m23a-xrslam-rosbag-preflight .
) 2>&1 | tee m23a_no_device_build.log
M23A_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23A_BUILD_EXIT_CODE=$M23A_BUILD_EXIT"
```

Require `M23A_JAZZY_ROSBAG_IMPORT_SELFTEST=PASS`,
`M23A_XRSLAM_REPLAY_INPUT_CONTRACT=PASS` (the same 21 tests),
`M23A_BUILD_EXIT_CODE=0`. If `rosbag2_py` is missing in the
existing base image, report that missing dependency. Do not silently
pull a new image or install tools over the network.

## Run synthetic ROS 2 bag roundtrip — no physical devices

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m23a-rosbag-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m23a-xrslam-rosbag-preflight
) 2>&1 | tee m23a_no_device_roundtrip.log
M23A_SIM_EXIT=${PIPESTATUS[0]}
echo "M23A_SIM_EXIT_CODE=$M23A_SIM_EXIT"
```

Require all:

```text
M23A_SYNTHETIC_ROSBAG_ROUNDTRIP=PASS
M23A_ROSBAG_WRONG_FRAME_INHIBIT=PASS
M23A_ROSBAG_MISSING_IMU_INHIBIT=PASS
M23A_ROSBAG_READER_SELFTEST=PASS
M23A_VERDICT=SYNTHETIC_BAG_READER_CONTRACT_ONLY
M23A_REAL_RECORDED_BAG_VALIDATED=FALSE
M23A_XRSLAM_TRACKING_VALIDATED=FALSE
M23A_DEVICE_ACCESS=NONE
M23A_SIM_EXIT_CODE=0
```

## Future candidate recorded bag, offline only

If a user-provided rosbag directory later becomes available, mount
that **data-only** directory read-only into a no-device container
and run the preflight CLI. Do not attach a camera or STM32 merely
to make this test possible. Input flags:

```text
--bag /path/to/existing/rosbag/directory
--mode monocular_imu
--storage-id sqlite3
--provenance user_declared_recorded
--camera-frame camera_optical_frame
--imu-frame imu_link
```

The bag storage plugin must already exist for the requested storage
format. On failure, CLI reports `M23A_BAG_PREFLIGHT_VERDICT=BLOCKED_REPLAY_INPUT`
and exits nonzero. The audit never asserts recorded-data authenticity.

## Next gate: actual XRSLAM replay (M23B)

A real-data bag must include complete serialized rectified image
payloads and correctly matched CameraInfo, not just M21I metadata.
Before starting XRSLAM as an isolated no-device process, provide
device-appropriate SLAM and sensor/noise YAML files and known camera/
IMU extrinsics, or keep pose/metric odometry explicitly disabled.
Any provisional offset is a TEST VALUE, not a measured calibration.
At M23B run image-only monocular first (no metric pose), then
image+IMU with pose_convention_validated=false; assert no /odom,
no map TF and no /cmd_vel. Confirm real XRSLAM frame-submission
receipts, tracking-status transitions and fail-closed dropouts;
do not infer tracking accuracy from input-contract PASS.

# M23J — Chronology of the first real camera/IMU recording

**Draft / no-device read-only analysis.** This branch is stacked on
M23I PR #103. Do not merge to main until #103 is merged or retargeted.
No MCU serial, physical camera, recorder, ROS node, XRSLAM, TF or motors.

## Accepted source evidence

M23H produced the first closed real bag, m23g_live_capture001.
M23I real-bag read-only audit preserved:
- 729 rectified Image, 3906 CameraInfo, 6001 IMU
- 729/729 image/CameraInfo matches; 1858 bounded info-cache evictions
- **158 image header gaps exceeding 250ms**
- longest image header gap **2951.988ms**
- 119.997594s three-stream header overlap, but **IMU fails
  coverage of the full image header-stamp window**
- M23A exact faults BLOCKED_IMAGE_STAMP_GAP and
  BLOCKED_NO_IMU_STAMP_COVERAGE; verdict remains BLOCKED, exit 2
- original DB SHA256:
  e9055fb76e4056023424bad44fdfd4b011bd1b46614e0f2d5535d10d641adb61
- original metadata SHA256:
  6ac930a2fdfdab649c0cb1961841609a8cad5abd354e8563760fd3e60687389f

M20 passive-observer count comparison:
camera_image 636, camera_info 3579, imu 6001.
The parallel rosbag2 subscriber saved 729, 3906, 6001.
This is **not evidence** the recorder alone loses the images;
different subscribers can drop differently. With much denser
CameraInfo and IMU, the rectified-image path is a major suspect.

## What M23J adds

- Read-only CDR scan from a CLOSED sqlite3 bag, using the same
  three exact ROS message types as M23A/M23D.
- Image-header and recorder-bag-receipt interval quantiles and
  histogram buckets; distinguishes header intervals from delivery
  intervals without pretending either measures physical exposure.
- The 8 largest image gaps with **time relative to first image**,
  camera-info and IMU header counts strictly inside each gap, and
  corresponding image bag-receipt gap.
- Signed exact IMU boundary offsets:
  \`first_imu_minus_first_image_ms\` positive => IMU begins too late;
  \`last_image_minus_last_imu_ms\` positive => IMU ends too early.
  Both must be <=0 for M23A's image-header window coverage.
  This does NOT calibrate camera exposure vs MCU acquisition phase.
- Bounded 100,000-message maximum per stream; pure-Python no-device
  unit tests; before/after M23E SHA-256 fingerprint.
- **No** change to M23A's 250ms image threshold, 300ms IMU
  threshold, recording, sensor source, calibration or motor control.
- A physically recorded bag can pass the *diagnostic chronology*
  and still be unsafe for VIO; neither verdict authorizes XRSLAM.

## Phase A — offline Python regressions

13 M23J + 10 M23I + 20 M23H + 19 M23G + 18 M23F + 19 M23E
+ 17 M23D + 21 M23A = **137 tests** expected PASS.

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m23j-real-bag-timeline
  git pull --ff-only origin feature/m23j-real-bag-timeline
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23j_real_bag_timeline_test.py
  python3 -u m23i_long_bag_diagnostics_test.py
  python3 -u m23h_bounded_capture_test.py
  python3 -u m23g_live_recording_preflight_test.py
  python3 -u m23f_existing_recording_inventory_test.py
  python3 -u m23e_offline_evidence_snapshot_test.py
  python3 -u m23d_recorded_bag_readiness_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23j_offline_contract.log
M23J_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23J_OFFLINE_EXIT_CODE=$M23J_OFFLINE_EXIT"
~~~

Require M23J_READ_ONLY_BAG_TIMELINE_CONTRACT=PASS,
M23J_250MS_CAMERA_GATE_UNCHANGED=PASS and exit 0.

## Phase B — build local, disconnected no-device image

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m23i-real-bag-forensics
  podman build --network none --no-cache \
    -f Dockerfile.m23j-timeline \
    -t leroy-sensors:m23j-real-bag-timeline .
) 2>&1 | tee m23j_image_build.log
M23J_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23J_BUILD_EXIT_CODE=$M23J_BUILD_EXIT"
~~~

Require M23J_JAZZY_READ_ONLY_IMPORT=PASS, 13/13 new tests
PASS, 10/10 M23I tests PASS and build exit 0.

## Phase C — no-device container test

~~~bash
set +e
set +o pipefail
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  localhost/leroy-sensors:m23j-real-bag-timeline \
  2>&1 | tee m23j_no_device_selftest.log
M23J_SIM_EXIT=${PIPESTATUS[0]}
echo "M23J_SIM_EXIT_CODE=$M23J_SIM_EXIT"
~~~

Require M23J_SYNTHETIC_TIMELINE_SELFTEST=PASS,
M23J_DEVICE_ACCESS=NONE and exit 0.

## Phase D — analyze SAME FIRST real bag READ-ONLY

Use the existing host directory, do not open a device, launch
leroy-ros, recapture or alter the original evidence. The mounted
bag's metadata and DB hashes must remain stable.

~~~bash
BAG_DIR="$M23H_HOST_RECORDINGS/m23g_live_capture001"
test -f "$BAG_DIR/metadata.yaml"

set +e
set +o pipefail
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --volume "$BAG_DIR":/input/recordings:ro \
  -e M23J_BAG=/input/recordings \
  localhost/leroy-sensors:m23j-real-bag-timeline \
  2>&1 | tee m23j_first_real_bag_timeline.log
M23J_REAL_BAG_EXIT=${PIPESTATUS[0]}
echo "M23J_REAL_BAG_EXIT_CODE=$M23J_REAL_BAG_EXIT"
~~~

Expected **M23J_VERDICT=BLOCKED_IMAGE_TIMING_OR_IMU_COVERAGE**
and exit **2** because 158 known image gaps violate the original
250ms gate and IMU header coverage is incomplete. An exit 2 here
is the **correct evidence classification**, not a software test
failure.

Collect M23J_TIMELINE_JSON fields:
- first_imu_minus_first_image_ms
- last_image_minus_last_imu_ms
- image_header_intervals_ms and image_bag_receipt_intervals_ms
- image_header_gap_buckets and image_header_gaps_over_250ms
- largest_image_header_gaps (with CameraInfo/IMU samples inside)
- sha256_manifest and bag_files_stable_during_audit

Do not infer physical time alignment or certify visual tracking
from stamped header offsets. This chronology determines whether to
focus next on IMU source startup/tail coverage, camera decode/
rectification performance, or DDS/subscriber backpressure.

# M23D: Offline recorded camera/IMU bag readiness

Status: source implemented, tests and real recorded-data audit PENDING.

## Grounded baseline

M23A #94/#95 validated the ROS 2 Jazzy CDR input contract. M23B
#96 merged to main at 93785ab0ccae9e6a1856d202f242a949c11cdf57
and demonstrated a real XRSLAM C++ monocular path ingesting fabricated
camera data while publishing no Odometry or dynamic TF. M23C #97 was
reported merged by the operator, but connected GitHub still returned
open/draft when M23D was created. M23D is therefore an INDEPENDENT
branch from confirmed M23B main and changes no M23C files.

No complete recorded real camera/CameraInfo/IMU image-payload rosbag
has been supplied. Historic M20 observations were aggregate sensor
ROS reports; M21I contains synthetic metadata only.

## What M23D does

- Run unchanged M23A sqlite3 CDR format preflight first, in
  monocular_imu mode, on EXISTING files only.
- Require ordinary metadata.yaml and .db3 files without symlink,
  absolute-path or parent-directory traversal references.
- Reopen the bag READ_ONLY for stream counts, maximal intra-stream
  message HEADER gaps, gaps over 250ms, header-time intersection, and
  a bounded 1024-byte-per-frame sampled image fingerprint.
- A passing structural inventory prints
  M23D_VERDICT=REPLAY_INPUT_RECORDING_PROFILE_ONLY.
  This does NOT prove acquisition timing, camera movement, image
  tracking quality, calibrated extrinsics or authenticated origin.
- Every result explicitly keeps calibration, tracking, motor control,
  physical device access and real-data provenance UNVERIFIED/FALSE.
  A user-declared "recorded" origin is just an unauthenticated label.

No publishers, ROS graph, XRSLAM runtime, Nav2, bridge, actuator,
serial access, camera capture or device mapping occurs here.

## Absolute physical safety rule

Opening the STM32 USB CDC /dev/ttyACM* may reset the MCU and energize
wheels/manipulator even without a motion command. Do NOT open it or
restart normal leroy-ros, change motor power or access live cameras.
Physical STOP acknowledgement is not validated. This PR confers
no authority to run the robot or move motors.

## Phase A: Arch offline Python only (NO devices)

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m23d-offline-recorded-bag-readiness
  git pull --ff-only origin feature/m23d-offline-recorded-bag-readiness
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23d_recorded_bag_readiness_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23d_offline_contract.log
M23D_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23D_OFFLINE_EXIT_CODE=$M23D_OFFLINE_EXIT"
~~~

Require M23D 17/17 and M23A 21/21 = 38/38 PASS,
M23D_RECORDED_READINESS_STATIC_CONTRACT=PASS,
M23D_REAL_RECORDED_BAG_VERIFIED=FALSE,
M23D_DEVICE_ACCESS=NONE, and M23D_OFFLINE_EXIT_CODE=0.

## Phase B: Build from existing local accepted M23B image

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m23b-xrslam-monocular-test
  podman build --network none --no-cache \
    -f Dockerfile.m23d-selftest \
    -t leroy-sensors:m23d-offline-bag-readiness .
) 2>&1 | tee m23d_no_device_build.log
M23D_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23D_BUILD_EXIT_CODE=$M23D_BUILD_EXIT"
~~~

Require M23D_JAZZY_ROSBAG_IMPORT=PASS and build exit 0.
Do NOT pull images or install dependencies from the network.

## Phase C: Synthetic Jazzy CDR smoke (NO devices)

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm --name leroy-m23d-offline-profile \
    --network none --ipc private --pid private \
    --cap-drop all --security-opt no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m23d-offline-bag-readiness
) 2>&1 | tee m23d_no_device_synthetic_profile.log
M23D_SIM_EXIT=${PIPESTATUS[0]}
echo "M23D_SIM_EXIT_CODE=$M23D_SIM_EXIT"
~~~

Required markers (not yet executed):

~~~text
M23D_SYNTHETIC_ROSBAG_PROFILE=PASS
M23D_BAG_PATH_TRAVERSAL_REJECTED=PASS
M23D_ROSBAG_READER_SELFTEST=PASS
M23D_VERDICT=SYNTHETIC_RECORDED_INPUT_PROFILE_ONLY
M23D_REAL_RECORDED_BAG_VERIFIED=FALSE
M23D_CALIBRATION_VALIDATED=FALSE
M23D_XRSLAM_TRACKING_VALIDATED=FALSE
M23D_DEVICE_ACCESS=NONE
M23D_SIM_EXIT_CODE=0
~~~

## Optional Phase D: Examine a pre-existing ordinary rosbag directory

Only after the software gates pass, and only if the user ALREADY has
a complete recorded image/CameraInfo/IMU sqlite3 bag. This DOES NOT
authorize new physical capture.

~~~bash
BAG_DIR="/absolute/path/to/existing/rosbag"
test -d "$BAG_DIR"
test -f "$BAG_DIR/metadata.yaml"
test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
podman run --rm \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --volume "$BAG_DIR":/input/bag:ro \
  -e M23D_BAG_PATH=/input/bag \
  localhost/leroy-sensors:m23d-offline-bag-readiness
~~~

Only bag files are mounted read-only, no /dev device access.
Passing format/profile status cannot prove origin or calibration.
Physical exposure-to-IMU offset, spatial/noise calibration, pose
direction, and genuine tracking accuracy remain independent gates.

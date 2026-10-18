# M23E: Read-only camera/IMU recording evidence snapshot

Status: source committed, operator-host acceptance PENDING.
There is no currently supplied complete real camera/Image/CameraInfo/Imu bag.

## Baseline

PR #97 M23C merged on 2026-10-18 at
73295be0f6920eb6c2b9b976c987d05fbcba152f.
PR #98 M23D merged at
73a4f508f9597b1a4b60be4f596cf739dcd309aa.
M23C: real XRSLAM C++ VIO accepted wholly fabricated camera/IMU CDR
messages and latched a synthetic 500ms IMU gap; metric output stayed
disabled. M23D: 38/38 Python offline tests and generated rosbag profile
plus metadata traversal rejection passed on operator host.

Historic real M20 ROS sensor observation is NOT a complete recorded
camera image/CameraInfo/IMU bag. M21I fixture is metadata only.

## Why M23E is distinct from M23D

M23D provides format and header-time profiling, but does not issue
a reproducible file manifest. M23E layers *on top*:

- SHA-256 of the EXACT metadata.yaml and listed sqlite3 .db3 files,
  bounded streaming 1MiB read chunks; rejects symlinks, FIFOs and
  nonregular file descriptors (O_NOFOLLOW where supported + fstat).
- Identical file-fingerprint snapshot before and after M23D's read and
  M23E's independent CDR scan. If bytes change, BLOCK the evidence.
- Per-topic count, min/max header intervals, min/max
  (rosbag receipt timestamp minus ROS message header timestamp),
  image encoding/dimension variants and CameraInfo rectified P
  projection changes. The result distinguishes observed numeric
  metadata consistency from actual hardware calibration.
- Calls the UNCHANGED M23D inspector; if its result is blocked or
  counts disagree, M23E blocks rather than overriding that result.
- Prints a machine-readable JSON manifest to **stdout only**, never
  writes to the pre-existing input bag, starts ROS nodes or records
  physical data. No camera or MCU interface is touched.

Passing M23E means ONLY OFFLINE_EVIDENCE_SNAPSHOT_ONLY:
the existing byte sequence was fingerprinted and its messages
profiled. SHA-256 proves neither original provenance nor source
identity. No sample proves physically measured exposure-to-MCU phase,
camera/IMU extrinsics, IMU noise model, intrinsic correctness,
XRSLAM tracking accuracy, pose/TF convention or physical safe stop.

All calibration, source identity, metric pose and physical motor
authority fields remain FALSE even when a user labels the source
user_declared_recorded. Only independent measurement/verification
can change those claims in a separate future milestone.

## Non-negotiable physical isolation

Opening the STM32 USB CDC /dev/ttyACM* alone can reset the MCU and
move wheels/manipulator, with NO command issued. NEVER open or mount
USB serial or physical camera devices, run robot.launch.py, start
production leroy-ros, create /cmd_vel publishers or test motors.
No verified physical STOP acknowledgement exists. All M23E tests use
only fabricated sensor records or a *pre-existing* data directory,
with rootless Podman --network none and NO --device mapping.

## Step A: Offline Arch Python tests (NO ROS or devices)

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m23e-offline-evidence-manifest
  git pull --ff-only origin feature/m23e-offline-evidence-manifest
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23e_offline_evidence_snapshot_test.py
  python3 -u m23d_recorded_bag_readiness_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23e_offline_contract.log
M23E_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23E_OFFLINE_EXIT_CODE=$M23E_OFFLINE_EXIT"
~~~

Expected: 19/19 new M23E + 17/17 M23D + 21/21 M23A = 57/57
PASS, M23E_OFFLINE_MANIFEST_CONTRACT=PASS,
M23E_REAL_RECORDED_BAG_VERIFIED=FALSE,
M23E_DEVICE_ACCESS=NONE and M23E_OFFLINE_EXIT_CODE=0.
These tests are synthetic memory and filesystem tests, not a ROS
bag runtime or physical provenance validation.

## Step B: Offline local image build

Requires the previously tested LOCAL M23D image.
Do not pull or download packages in the build.

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m23d-offline-bag-readiness
  podman build --network none --no-cache \
    -f Dockerfile.m23e-selftest \
    -t leroy-sensors:m23e-evidence-snapshot .
) 2>&1 | tee m23e_build.log
M23E_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23E_BUILD_EXIT_CODE=$M23E_BUILD_EXIT"
~~~

Require M23E_OFFLINE_MANIFEST_CONTRACT=PASS,
M23E_JAZZY_EVIDENCE_IMPORT=PASS, M23E_BUILD_EXIT_CODE=0.

## Step C: Isolated synthetic CDR and tamper smoke test

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm --name leroy-m23e-fake-evidence-only \
    --network none --ipc private --pid private \
    --cap-drop all --security-opt no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m23e-evidence-snapshot
) 2>&1 | tee m23e_synthetic_cdr.log
M23E_SIM_EXIT=${PIPESTATUS[0]}
echo "M23E_SIM_EXIT_CODE=$M23E_SIM_EXIT"
~~~

Expected (NOT yet observed):

~~~text
M23E_SYNTHETIC_CDR_EVIDENCE_SNAPSHOT=PASS
M23E_SYNTHETIC_MANIFEST_MUTATION_DETECTED=PASS
M23E_JAZZY_CDR_EVIDENCE_SELFTEST=PASS
M23E_VERDICT=SYNTHETIC_EVIDENCE_HASHING_ONLY
M23E_REAL_RECORDED_BAG_VERIFIED=FALSE
M23E_CALIBRATION_VALIDATED=FALSE
M23E_XRSLAM_TRACKING_VALIDATED=FALSE
M23E_DEVICE_ACCESS=NONE
M23E_SIM_EXIT_CODE=0
~~~

The mutation negative test appends bytes ONLY to a disposable
synthetic .db3 file in /tmp after the normal read. It never changes
an actual user file. No ROS graph or XRSLAM executable is started.

## Optional Step D: EXISTING real recorded bag, file access only

Do not run this step without an already-available sqlite3 ROS 2
bag whose metadata references complete rectified Image,
CameraInfo and IMU data files. Do not attempt new physical capture.
Use an absolute BAG_DIR containing metadata.yaml and .db3.

~~~bash
BAG_DIR="/absolute/path/to/existing/rosbag"
test -d "$BAG_DIR"
test -f "$BAG_DIR/metadata.yaml"
test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
podman run --rm --name leroy-m23e-read-only-existing-bag \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --volume "$BAG_DIR":/input/bag:ro \
  -e M23E_BAG_PATH=/input/bag \
  localhost/leroy-sensors:m23e-evidence-snapshot
~~~

This prints M23E_EVIDENCE_JSON containing the SHA-256 manifest,
M23D profile and M23E timing/CameraInfo summaries. A profile may
BLOCK. If it does, report the exact faults, do not bypass them.

The output's M23E_VERDICT=OFFLINE_EVIDENCE_SNAPSHOT_ONLY merely
means the file bytes and timing were inventoried, not that data is
actually real or suitable for calibrated metric VIO. Independently
retain external provenance and calibration evidence; no motor
motion is enabled by a bag-profile result.

## Outstanding real-world gates

Full pre-existing real CDR image payload recording; measured
camera exposure to IMU acquisition offset; measured body/camera/IMU
extrinsics; verified noise/camera intrinsics; real XRSLAM track-loss
and scale/pose checks; and verified physical STOP/motor-power safety.
M23E supplies a deterministic artifact to refer to, NOT a substitute.

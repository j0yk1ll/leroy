# M23F: Strictly offline inventory of existing camera/IMU recordings

**Status: source implemented; operator validation PENDING.**
No complete real Image/CameraInfo/Imu bag has been supplied.

## Verified baseline

PR #99 M23E merged at edb20302281194dca5d483e8211ffc03059e8915.
Operator reported **57/57 offline regressions PASS** and isolated Jazzy
CDR evidence snapshot and SHA256 mutation test PASS. PR #97 M23C
native synthetic VIO/IMU fault latch and PR #98 M23D recorded-input
profiling were merged earlier. No real image payload recording or
physical camera–IMU alignment was established by those synthetic tests.
No .db3 or .mcap recordings are checked into the tracked repository.

## New scope: selection of ACTUALLY EXISTING offline files

M23F is an inventory and triage wrapper around the unchanged M23E
evidence snapshot, which itself invokes M23D/M23A checks.

- Caller supplies one explicit directory. Inventory examines only
  that directory or its immediate subdirectories. **No recursion**,
  no automatic filesystem, production ROS or connected-device search.
- Reject symlink top-level root, system/device filesystem roots,
  and any symlink storage/metadata or unsafe manifest path. Limit
  inspection to 256 immediate entries and 32 rosbag candidates.
- Without Jazzy, an optional metadata-reference-only scan verifies
  manifest-listed regular files exist; it does NOT deserialize sensors.
- With --audit-cdr in disconnected Podman, candidate directories
  undergo full M23E SHA-256, M23D timing and M23A format audits.
  A candidate PASS is only OFFLINE_EVIDENCE_SNAPSHOT_ONLY, with source
  authenticity and real VIO suitability still UNVERIFIED.
- If the root contains no metadata.yaml candidates, M23F reports
  **BLOCKED_NO_EXISTING_ROSBAG**, exits 3, and never pretends a
  recorded dataset exists.
- An unsafe or invalid candidate BLOCKS the inventory root rather
  than being ignored in favor of any other apparently valid candidate.
- Header stamps, CameraInfo, file paths, image bytes and file hashes
  can be inspected as offline evidence, never physical calibration.

## Critical physical boundary

Opening the real STM32 USB CDC device /dev/ttyACM* can reset firmware
and energize wheels or manipulator with zero movement command.
NEVER open/mount USB serial or camera devices, start production
leroy-ros, launch robot.launch.py, send /cmd_vel, enable map TF,
or claim a physical STOP ACK. No motor or physical sensor access is
permitted. A passing M23F inventory is NOT motor, real tracking,
camera–IMU offset, spatial calibration or provenance acceptance.

## Phase A — Host Arch, no ROS, no devices

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m23f-existing-recording-inventory
  git pull --ff-only origin feature/m23f-existing-recording-inventory
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23f_existing_recording_inventory_test.py
  python3 -u m23e_offline_evidence_snapshot_test.py
  python3 -u m23d_recorded_bag_readiness_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23f_offline_contract.log
M23F_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23F_OFFLINE_EXIT_CODE=$M23F_OFFLINE_EXIT"
~~~

Require **75/75** = 18 M23F + 19 M23E + 17 M23D + 21 M23A.
Require M23F_OFFLINE_INVENTORY_CONTRACT=PASS,
M23F_REAL_RECORDED_DATA_FOUND=FALSE,
M23F_DEVICE_ACCESS=NONE, and M23F_OFFLINE_EXIT_CODE=0.
These tests use temporary synthetic filesystem and in-memory records.

## Phase B — No-network local-image build

Use the previously accepted local M23E image, NOT a remote pull:

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m23e-evidence-snapshot
  podman build --network none --no-cache \
    -f Dockerfile.m23f-selftest \
    -t leroy-sensors:m23f-existing-recording-inventory .
) 2>&1 | tee m23f_no_device_build.log
M23F_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23F_BUILD_EXIT_CODE=$M23F_BUILD_EXIT"
~~~

Require M23F_OFFLINE_INVENTORY_CONTRACT=PASS,
M23F_JAZZY_INVENTORY_IMPORT=PASS and M23F_BUILD_EXIT_CODE=0.

## Phase C — Isolated synthetic Jazzy CDR contract

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m23f-synthetic-inventory \
    --network none --ipc private --pid private \
    --cap-drop all --security-opt no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m23f-existing-recording-inventory
) 2>&1 | tee m23f_synthetic_inventory.log
M23F_SIM_EXIT=${PIPESTATUS[0]}
echo "M23F_SIM_EXIT_CODE=$M23F_SIM_EXIT"
~~~

Require ALL:

~~~text
M23F_EMPTY_RECORDING_INVENTORY_BLOCKED=PASS
M23F_SYNTHETIC_CDR_CANDIDATE_INVENTORY=PASS
M23F_UNSAFE_RECORDING_CANDIDATE_BLOCKED=PASS
M23F_JAZZY_CDR_INVENTORY_SELFTEST=PASS
M23F_VERDICT=SYNTHETIC_INPUT_INVENTORY_ONLY
M23F_REAL_RECORDED_DATA_FOUND=FALSE
M23F_CALIBRATION_VALIDATED=FALSE
M23F_XRSLAM_TRACKING_VALIDATED=FALSE
M23F_DEVICE_ACCESS=NONE
M23F_SIM_EXIT_CODE=0
~~~

Synthetic Jazzy temporary test files are written ONLY to /tmp inside
the disposable container; no real data is touched, and the test
does not authenticate any recording.

## Phase D — Optional real-existing-data inventory (NOT capture)

**Only if an existing ordinary offline directory is already known.**
Choose an explicit host path containing one or more rosbag2
directories, each holding metadata.yaml and .db3, or a single bag
directory. Never use /, /dev, /proc, /sys, a live ROS mount or
physical device path. The complete source must pre-exist; M23F
does not acquire anything.

~~~bash
BAG_ROOT="/absolute/path/to/pre-existing-offline-recordings"
test -d "$BAG_ROOT"
test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
set +e
set +o pipefail
podman run --rm \
  --name leroy-m23f-existing-recordings \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --volume "$BAG_ROOT":/input/recordings:ro \
  -e M23F_DATA_ROOT=/input/recordings \
  localhost/leroy-sensors:m23f-existing-recording-inventory \
  2>&1 | tee m23f_existing_recordings.log
M23F_REAL_INPUT_EXIT=${PIPESTATUS[0]}
echo "M23F_REAL_INPUT_EXIT_CODE=$M23F_REAL_INPUT_EXIT"
~~~

This produces M23F_INVENTORY_JSON and one of the following:
- BLOCKED_NO_EXISTING_ROSBAG (exit 3): missing recorded input.
- BLOCKED_CANDIDATE_STORAGE_OR_CDR (exit 2): unsafe or invalid bag.
- EXISTING_CDR_INPUTS_PROFILED_UNVERIFIED (exit 0): only existing
  file and ROS message format audited, physical source still NOT
  authenticated or calibrated.

An exit 0 does not mean that a file is real rather than synthetic;
the code intentionally always reports:
recorded_sensor_origin_verified=false,
recorded_sensor_calibration_verified=false,
real_xrslam_tracking_validated=false,
safe_for_metric_vio=false,
physical_motion_authorized=false, device_access=false.

A root containing no bags is a useful BLOCKER result. It is NOT a
reason to open the STM32, capture live sensor data, or enable motion.
A later physically authorized and independent calibration campaign
would be needed; that authorization is outside M23F.

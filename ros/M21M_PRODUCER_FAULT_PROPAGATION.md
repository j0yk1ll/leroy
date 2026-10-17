# M21M — Reject joined ROS/IMU producer faults in metadata lifecycle

## Prior evidence and review stack

Operator M21J admission/STOP tests: 10/10 PASS, offline exit 0.
Operator M21K quiescence tests: 9/9 PASS, offline exit 0.
Operator M21L isolated ROS lifecycle test: PASS, 12 SYNTHETIC events,
four each per stream, zero active producer threads, schema accepted,
no physical devices, simulated exit 0.

Draft PRs are stacked in order: M21J #75 against main, M21K #76
against M21J, M21L #77 against M21K, M21M against M21L.
None of these drafts grants approval for physical serial or camera use.

## Additional fail-closed case

M21L checks `errors` in its outer selftest, but M21K's lifecycle helper
could report SYNTHETIC_STOP_QUIESCE_SAVE_CONTRACT when every thread
was joined, STOP callback returned true and JSONL saved EVEN IF a
producer recorded an exception before exiting. The lifecycle API now
accepts optional `producer_faults` (a list or tuple), examines it
only after registered workers have joined, reports producer_fault_count
and producer_faults, and rejects acceptance when any are present.

This is a functional correctness guarantee only for reported faults
from REGISTERED producer threads. Hidden exceptions, unregistered
ROS workers, process death and unacknowledged real STOP are not covered.
Joined producers with faults may still yield a valid diagnostic JSONL
file, but the lifecycle verdict remains STOP_QUIESCENCE_OR_SAVE_UNVERIFIED.

The actual M21L simulated ROS executor test now passes its shared
errors list into the lifecycle API rather than relying only on
a separate outer error check. The existing M21K default no-fault
behavior is preserved.

## Test coverage

Five pure-Python tests cover: already-terminated camera fault despite
valid JSONL, multiple IMU/camera errors, empty errors preserving success,
invalid fault collection rejecting before simulated STOP, and a worker
that reports a fault just before it exits during the join.

The derived ROS test image inherits the operator-tested M21L simulator.
During build it compiles patched source and runs nine existing M21K
lifecycle tests. During run it executes the five negative fault tests
and the revised M21L ROS simulated positive end-to-end test.

## Arch build — no physical devices or production container

Run in Bash from the Leroy checkout. Existing local deletions in
ros/CAMERA_TIMESTAMP_FIX.md and utils/src/utils/__init__.py must
not be discarded or modified as a side effect of testing.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m21m-producer-fault-propagation
  git pull --ff-only origin feature/m21m-producer-fault-propagation
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m21l-ros-quiescence-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m21m-selftest \
    -t leroy-sensors:m21m-producer-fault-selftest .
) 2>&1 | tee m21m_no_device_build.log
M21M_BUILD_EXIT=${PIPESTATUS[0]}
echo "M21M_BUILD_EXIT_CODE=$M21M_BUILD_EXIT"
```

Expected M21M_FAULT_HOOK_IMPORT_SELFTEST=PASS, 9 M21K build tests OK,
M21M_BUILD_EXIT_CODE=0.

## Run fault injection + ROS callbacks in isolated no-device container

Only if build succeeded:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m21m-producer-fault-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m21m-producer-fault-selftest
) 2>&1 | tee m21m_no_device_simulation.log
M21M_SIM_EXIT=${PIPESTATUS[0]}
echo "M21M_SIM_EXIT_CODE=$M21M_SIM_EXIT"
```

Expected: five negative tests OK and
M21M_PRODUCER_FAULT_SELFTEST=PASS;
M21M_VERDICT=SYNTHETIC_PRODUCER_FAULT_REJECTION;
M21L_ROS_QUIESCENCE_SELFTEST=PASS;
M21M_ROS_FAULT_INTEGRATION_SELFTEST=PASS;
M21M_DEVICE_ACCESS=NONE; M21M_SIM_EXIT_CODE=0.

## Physical safety boundaries

These are simulated ROS camera nodes and entirely in-memory MCU bytes.
The image has no /dev/video or /dev/ttyACM0 mapping, performs NO
physical sensor capture, motor actuation, Nav2, XRSLAM, TF, odometry
or DepthART. Leroy's production leroy-ros container must stay exited.

Real MCU CDC open can spuriously energize motors. Before any separately
approved physical sensor test, BOTH wheel and manipulator motor power
must be physically isolated, a hardware E-stop engaged and verified,
robot immobilized, device identity and exclusive ownership checked.
Any uncertainty about physical MCU STOP ACK means motor power remains
physically inhibited. No physical exposure-to-IMU offset or spatial
extrinsics have been measured. DepthART CPU optimization remains deferred.

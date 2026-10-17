# M21G — No-device IMU metadata callback terminal-fault regression

## Operator evidence

M21F source-level metadata simulation accepted on Arch:
M21F_SOURCE_METADATA_SELFTEST=PASS, 33 synthetic events, M21C schema valid,
M21F_DEVICE_ACCESS=NONE, M21F_SIM_EXIT_CODE=0. PR #71 verified merged
into main at 6d05e0d72334c6ef4bd1d8fdb41d0013961baab7.
User provided the simulation output, not an explicit M21F build exit code.

## Fault identified and change

The optional M21F post-publish metadata hook already latches
terminal_fault=imu_metadata_callback_failed if the callback raises, but
ImuStreamSource.consume did not previously check terminal_fault at entry.
A caller catching the error could therefore allow subsequent publication
after the metadata path had failed. M21G adds an entry guard:

    if self.terminal_fault is not None:
        raise RuntimeError('imu_source_terminal_fault_latched: ' + ...)

This guard applies after ANY latched source fault, not only callbacks.
Existing M18/M20 callers using the default on_imu_published=None behave
identically on the normal successful message path.

**Crucial timing nuance:** The callback executes AFTER a successful ROS
publish and after the source increments the successful publish counter.
Therefore the single packet that triggers a callback failure may have
already appeared on /imu/data_raw without a corresponding metadata row.
The guard blocks FUTURE consumption, not retroactive publication. A physical
runner's existing exception cleanup attempts serial STOP but cannot
guarantee an ACK after device loss, abrupt process death or timeout.

## Six new synthetic ROS fault-path tests

m21g_fault_latch_selftest.py drives the actual M18 parser and ROS publisher
using fake in-memory MCU ASCII strings. It checks the unchanged default
no-hook path, callback exception latching and future publish refusal,
bounded recorder overflow causing the same latch, malformed/calibration-
withheld packets not invoking callbacks, other prelatched source faults,
and construction with a noncallable observer rejected.

These tests DO NOT open /dev/ttyACM0, /dev/video2, V4L2, a USB serial
device or the normal motor-capable leroy-ros container. No artificial
claim is made about real camera–IMU physical synchronization.

## Arch: build device-free M21G image

Run from the Leroy Git checkout in Bash. Do not set interactive global
strict mode. No --device flags or host USB access is used.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21g-imu-hook-fault-latch
  git pull --ff-only origin feature/m21g-imu-hook-fault-latch
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m21f-source-hook-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m21g-selftest \
    -t leroy-sensors:m21g-fault-latch-selftest .
) 2>&1 | tee m21g_fault_latch_build.log
M21G_BUILD_EXIT=${PIPESTATUS[0]}
echo "M21G_BUILD_EXIT_CODE=$M21G_BUILD_EXIT"
```

Require M21G_SOURCE_IMPORT_SELFTEST=PASS and M21G_BUILD_EXIT_CODE=0.

## Run device-free ROS fault-path simulation

Only after build exit 0:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m21g-fault-latch-only \
    --network none --ipc private --pid private \
    --shm-size 512m \
    --cap-drop all --security-opt no-new-privileges \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m21g-fault-latch-selftest
) 2>&1 | tee m21g_fault_latch_simulation.log
M21G_SIM_EXIT=${PIPESTATUS[0]}
echo "M21G_SIM_EXIT_CODE=$M21G_SIM_EXIT"
```

Require: Ran 6 tests, OK, all of:

    M21G_SOURCE_FAULT_SELFTEST=PASS
    M21G_VERDICT=SIMULATED_TERMINAL_FAULT_LATCH_CONTRACT
    M21G_DEVICE_ACCESS=NONE
    M21G_SIM_EXIT_CODE=0

## Physical test authorization: NONE

These simulated failures do not prove serial STOP ACK on real STM32
USB loss or measured physical acquisition offset. The production leroy-ros
container must remain exited and no motor, Nav2, XRSLAM, odom/TF or
DepthART processes may be started under this test. Future physical MCU
access requires distinct operator authorization and physical isolation
of BOTH wheel and manipulator power, a verified hardware E-stop,
immobilized robot and verified current device identities/ownership.
Missing STOP ACK requires keeping motor power physically inhibited.
DepthART CPU optimization remains deferred.

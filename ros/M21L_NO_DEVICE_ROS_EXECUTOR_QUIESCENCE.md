# M21L — No-device ROS executor and source producer quiescence self-test

## Verified prerequisites

M21J: 10/10 offline tests, admission ready at 20,000-event capacity;
source-level simulated STOP-before-save. Operator M21K: 9/9 tests passed,
nine synthetic events saved, correct STOP/quiesce/join/save contract, exit 0.

M21J draft PR #75 targets main. M21K draft PR #76 is STACKED on M21J.
M21L branches from M21K and must be reviewed after both predecessors.

## What the ROS self-test does

The test uses the genuine PassiveCameraMetadataNode ROS subscriptions,
two fake camera publisher nodes (rectify_node and usb_cam), the real
M18 ImuStreamSource parsing IN-MEMORY ASCII simulated MCU packets, and
a shared MetadataCapture with SYNTHETIC provenance.

It starts three explicitly registered worker threads:
1. rclpy SingleThreadedExecutor spinning only the passive camera node
2. synthetic image and CameraInfo publisher
3. fake MCU telemetry source pushing through the M18 parser and tap

After all three streams record at least four events each, M21K's
finish_after_quiescence invokes a FAKE STOP callback, asks every
worker to exit, joins all three threads with a bounded deadline, and
only then validates and serializes the metadata JSONL.

It refuses acceptance if any registered producer remains alive, an
exception is reported, the output is invalid, or M21K rejects lifecycle.
The test never starts actual camera acquisition, opens the MCU CDC port,
runs the M20 physical sensor launcher or enables motors.

Graph publisher names are only simulated; they do not identify any real
sensor. The test does not prove that an unregistered thread or a second
executor could not append data. Physical camera–IMU offset, extrinsics,
STM32 STOP ACK, and any robot motion state remain UNVALIDATED.

## Arch host: isolated no-device build

Run from the Git checkout. Previous `git status` showed deletions of
ros/CAMERA_TIMESTAMP_FIX.md and utils/src/utils/__init__.py. Do not
restore or discard those local changes as part of this test.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m21l-no-device-ros-executor-quiescence
  git pull --ff-only origin feature/m21l-no-device-ros-executor-quiescence
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m21h-cleanup-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m21l-selftest \
    -t leroy-sensors:m21l-ros-quiescence-selftest .
) 2>&1 | tee m21l_no_device_build.log
M21L_BUILD_EXIT=${PIPESTATUS[0]}
echo "M21L_BUILD_EXIT_CODE=$M21L_BUILD_EXIT"
```

Require M21L_ROS_IMPORT_SELFTEST=PASS, M21L_BUILD_EXIT_CODE=0.

## Run the disposable, no-device ROS test container

Only if image build succeeds:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m21l-ros-quiescence-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m21l-ros-quiescence-selftest
) 2>&1 | tee m21l_no_device_simulation.log
M21L_SIM_EXIT=${PIPESTATUS[0]}
echo "M21L_SIM_EXIT_CODE=$M21L_SIM_EXIT"
```

Require M21L_ROS_QUIESCENCE_SELFTEST=PASS,
M21L_VERDICT=SIMULATED_EXECUTOR_AND_SOURCE_JOIN_ONLY,
M21L_DEVICE_ACCESS=NONE and M21L_SIM_EXIT_CODE=0.
The M21L_LIFECYCLE_REPORT_JSON must show
`stop_acknowledged=true`, `producers_quiescent=true`,
`metadata_saved=true`, `evidence_type=SYNTHETIC`,
`physical_stop_acknowledgement_validated=false`.
The M21L_SCHEMA_REPORT_JSON must validate the M21C format with
at least four messages each for camera_image, camera_info and imu.

## Physical boundary remains unchanged

Do not map /dev/ttyACM0 or /dev/video8 into this container. Never start
motor-capable leroy-ros or run XRSLAM, Nav2, TF, odometry, DepthART or
motor bridge. Any future real STM32 serial access requires separate
operator authorization, BOTH wheel and manipulator power physically
inhibited, hardware E-stop verified, robot immobilized, exact current
device identities and ownership checked, and isolated least-privilege
sensor-only container. Unverified STOP ACK requires motor power to remain
physically inhibited. CPU DepthART optimization remains deferred.

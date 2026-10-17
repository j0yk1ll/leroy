# M21E — Isolated ROS camera-metadata adapter selftest (NO DEVICE ACCESS)

## Background

M20 accepted real camera+MCU IMU ROS receipt over 120 seconds, without
claiming physical camera exposure / IMU acquisition synchronization.
M21A offline audit passed. M21B synthetic lag recovery passed. M21C metadata
schema passed. M21D offline bounded recorder passed 13/13 tests and emitted
33 synthetic records; PR #69 corrected a false-positive comment-string guard
and has been verified merged.

M21E verifies that the M21D passive ROS camera adapter can read simulated
message callbacks through a real ROS 2 Jazzy executor under the same
isolated ROS domain used for M20. Fake ROS publisher nodes are named
/camera/rectify_node and /camera/usb_cam, and emit EMPTY-pixel synthetic
Image messages and CameraInfo messages. The source-level IMU helper receives
artificial MCU seq, t_us and gyro fields. No STM32 interface is opened, and
these are NOT physical measurements or authenticated publisher identities.

Existing M20 physical sensor runner is NEVER invoked by the M21E image.
The M21E test image inherits the operator-validated M20 ROS runtime solely
to reuse ROS Jazzy and the existing camera/IMU contract Python modules.
The M21E runner refuses mapped /dev/video2 or /dev/ttyACM0 devices.

## Operator: hardware-free image build and selftest

Run from the Leroy Git checkout on Arch, not inside the motor-capable
production leroy-ros container. Bash only; no interactive global strict
mode changes. The source branch must be checked out before build.

```bash
set +e
set +u
set +o pipefail

(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21e-isolated-ros-adapter-selftest
  git pull --ff-only origin feature/m21e-isolated-ros-adapter-selftest
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m20-camera-imu
  podman build --network none --no-cache \
    -f Dockerfile.m21e-selftest \
    -t leroy-sensors:m21e-ros-selftest .
) 2>&1 | tee m21e_isolated_image_build.log
M21E_BUILD_EXIT=${PIPESTATUS[0]}
echo "M21E_BUILD_EXIT_CODE=$M21E_BUILD_EXIT"
```

Require M21E_BUILD_EXIT_CODE=0, 13 passing imported offline M21D tests,
and M21E_ROS_IMPORT_SELFTEST=PASS. Build has no device access.

Only after successful build:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m21e-simulated-ros-only \
    --network none --ipc private --pid private \
    --shm-size 512m \
    --cap-drop all \
    --security-opt no-new-privileges \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m21e-ros-selftest
) 2>&1 | tee m21e_simulated_ros_selftest.log
M21E_SIM_EXIT=${PIPESTATUS[0]}
echo "M21E_SIM_EXIT_CODE=$M21E_SIM_EXIT"
```

Require both M21E_ROS_ADAPTER_SELFTEST=PASS and
M21E_TEST_VERDICT=SIMULATED_ROS_METADATA_CONTRACT_ONLY, plus exit 0.
Report must record 33 synthetic metadata events, 11 each from simulated
camera image, CameraInfo and artificial source-level IMU packets.
Even with an accepted format, physical_camera_imu_offset_validated=false.
No physical camera, MCU, E-stop or motion command is used in this test.

## Engineering interpretation

The test demonstrates ROS graph discovery, callback reception, an import-only
adapter, no-pixel record storage, exact allowed simulated publishers, and
a source-level simulated MCU packet helper under the real ROS stack.
It does not demonstrate actual patched camera throughput, source authenticity,
STM32 serialization/STOP ACK cleanup or a defensible exposure–IMU offset.
M21D's ROS adapter still is not wired into the M18 physical source, nor is
there a sensor-only physical launcher for metadata recording.

No authorization is given for another physical run. Before future physical
MCU access, BOTH wheel and manipulator power must be physically inhibited,
verified hardware E-stop engaged, robot immobilized, production leroy-ros
exited and current device identity/ownership checked. Any approved run must
use isolated least-privilege Podman with verified devices only and the exact
approved MCU START/STOP allowlist. Missing STOP ACK leaves MCU state unknown.
No normal motor bridge, /cmd_vel, XRSLAM odom/TF, Nav2 or DepthART.
DepthART CPU optimization remains deferred.

# M21F — No-device M18 source-level IMU metadata hook self-test

## Verified prerequisite

M20 physically observed the USB camera and MCU IMU concurrently for 120s;
M21A/B/C/D passed offline checks, and M21E passed operator-run build and
simulated ROS adapter with 33 SYNTHETIC metadata records, both exit codes 0.
PR #70 is verified merged into main at 77f7d2250518b305496a702b33e6fc77b2bd7146.

## Scope of this change

This change adds one optional callback argument to M18's ImuStreamSource:

    ImuStreamSource(on_imu_published=None)

Existing M18 and M20 callers do not provide it. No serial command, sensor
header, publisher topic, default launch, normal bridge or safety contract
is changed when the optional callback is unset. If configured, it executes
ONLY after a successfully published calibrated ROS Imu packet and receives
(copy_of_parsed_mcu_fields, mapped_ros_stamp_ns, receipt_monotonic_ns).
If the callback raises, the source latches imu_metadata_callback_failed
and raises RuntimeError, allowing the owning physical runner's finally
block to attempt STOP (not guaranteed if the process is killed).

The new m21f_imu_source_hook_selftest.py drives the REAL M18 parser,
McuClockSync and ImuStreamSource.publish method using generated MCU-format
byte strings in memory. It verifies source MCU seq/t_us, matches
published ROS Imu header stamps to recorded JSONL metadata and excludes
malformed and calibration-withheld packets. Camera/ImageInfo publishers
are SIMULATED as in M21E, using the existing passive adapter.

Absolutely NO actual /dev/ttyACM0 or /dev/video2; no USB opening,
serial START/STOP requests, camera acquisition, locomotion, motors, odom/TF,
XRSLAM, Nav2, or DepthART. The derived test image inherits the already-built
localhost/leroy-sensors:m21e-ros-selftest image but overrides its command.

## Arch Linux build — completely hardware-free

Run from the Leroy Git checkout. Do not start/restart the production
motor-capable leroy-ros container. Do not globally set interactive strict
Bash mode. No --device options appear anywhere below.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21f-no-device-imu-source-tap
  git pull --ff-only origin feature/m21f-no-device-imu-source-tap
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m21e-ros-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m21f-selftest \
    -t leroy-sensors:m21f-source-hook-selftest .
) 2>&1 | tee m21f_source_hook_build.log
M21F_BUILD_EXIT=${PIPESTATUS[0]}
echo "M21F_BUILD_EXIT_CODE=$M21F_BUILD_EXIT"
```

Require M21F_BUILD_EXIT_CODE=0 and
M21F_M18_HOOK_IMPORT_SELFTEST=PASS. No physical devices are accessible.

## Run no-device ROS simulation

Only after build exit 0:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m21f-source-hook-only \
    --network none --ipc private --pid private \
    --shm-size 512m \
    --cap-drop all --security-opt no-new-privileges \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m21f-source-hook-selftest
) 2>&1 | tee m21f_source_hook_simulation.log
M21F_SIM_EXIT=${PIPESTATUS[0]}
echo "M21F_SIM_EXIT_CODE=$M21F_SIM_EXIT"
```

Require both M21F_BUILD_EXIT_CODE=0 and M21F_SIM_EXIT_CODE=0 plus:

    M21F_SOURCE_METADATA_SELFTEST=PASS
    M21F_TEST_VERDICT=SIMULATED_IMU_SOURCE_HOOK_ONLY
    M21F_DEVICE_ACCESS=NONE

The JSON report must have SYNTHETIC provenance, a valid M21C schema,
33 events (11 simulated image, 11 CameraInfo and 11 actual-publisher-path
IMU messages from in-memory fake telemetry), and
physical_camera_imu_offset_validated=false.

## Non-claims / next engineering gate

The source hook is tested only on fake serial-format bytes, not actual MCU
USB traffic. The M21F image has NO camera/MCU devices and does not execute
the M18 or M20 physical runner. Synthetic metadata does not prove source
authenticity, true camera exposure timing, absolute MCU phase, calibrated
camera-IMU extrinsics, optical tracking or a safe motion system.

Before physical recording, the changed M18 source must undergo isolated
device-free fault-path regression in Jazzy and a separate M20-style physical
runner review. The new callback could add synchronous work to the IMU
source, so its latency, boundedness and failure behavior must be verified.
Any later physical use needs explicit operator authorization: physically
inhibit BOTH wheel and manipulator power, verify hardware E-stop engaged,
immobilize robot, confirm USB identities and exclusive ownership, keep
leroy-ros exited and use only network-none, device-minimal, least-privilege
Podman. Missing serial STOP ACK means MCU state is unverified and motor
power must remain isolated. Do not enable XRSLAM, Nav2, TF, odometry,
DepthART or motor control. DepthART CPU optimization remains deferred.

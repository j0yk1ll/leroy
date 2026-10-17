# M20 — Combined real front-camera + timestamped IMU ROS message contract

## Aim and prior evidence

M16 accepted a **real** front camera + CPU DepthART message contract in
a 45-second sensor-only sidecar. M18 accepted **real STM32 MCU IMU**
(`seq` + `t_us`) at 50 Hz for 30 seconds, 1500/1500 clean.
M19 accepted a **120-second** IMU-only clock-rate audit:
6000/6000 clean, rate estimator ready and fitted **-39 ppm** host-relative.
M17 accepted 30/30 basic firmware IMU queries. All were **partial,
distinct** sensor results; overall full M16 navigation integration
remained incomplete.

M20 combines ONLY the **front camera** (`/camera/image_rect` and
`/camera/camera_info`) and **MCU-timestamped IMU**
(`/imu/data_raw`) within a single isolated ROS 2 domain **170**.
It runs for 120 seconds while the robot is stationary and physically
prevented from moving. The camera launch includes only the existing
patched `usb_cam` and `image_proc::RectifyNode`.
**DepthART is not started:** CPU performance work was explicitly
deferred; it is not relevant to establishing these two sensor inputs'
concurrent ROS transport. XRSLAM, mapping, odom, TF and navigation
remain off.

M20 reuses without modification:
- the actual M18 `ImuStreamSource` and `McuClockSync`, which
  publish only `sensor_msgs/msg/Imu`, BEST_EFFORT QoS, `imu_link`;
- the same M18 exact serial write allowlist:
  `set_imu_stream_hz 50\r\n` on start and
  `set_imu_stream_hz 0\r\n` on cleanup, and MCU ACK checks;
- M16's actual nine-topic `PassiveSensorObserver`, whose
  existing graph warning **will correctly identify** the isolated
  IMU publisher `m18_imu_source`, not normal `leroy_bridge`;
- the existing camera intrinsic YAML; its physical accuracy is
  NOT separately validated here.

The observer spins in a dedicated ROS thread while the main thread
reads the MCU serial device. This prevents serial polling from
blocking camera and IMU observation callbacks.

M20 reports:
```text
M20_IMU_STREAM_START_ACK=PASS
M20_IMU_STREAM_STOP_ACK=PASS
M16_PASSIVE_REPORT_JSON=...
M16_VERDICT=INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS
M20_COMBINED_REPORT_JSON=...
M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT
```

**M20 acceptance** requires complete 120 seconds, both IMU stream
ACKs, no source/sequence/timing faults, clock-rate mapper ready,
~50 Hz MCU/ROS IMU with >=5700 source packets, >=5600 accepted
observer IMU messages, >=400 accepted camera rectified images and
CameraInfo each, zero invalid messages, correct frame IDs
(`camera_optical_frame`, `imu_link`), exact observed publisher
provenance, 640x480 rectified RGB and camera info, final valid fresh
receipts, and at least 90 seconds of **intersecting monotonic
message-receipt windows** for all three topics. Forbidden ROS graph
publishers/nodes or unexpected depth, XRSLAM, odom, TF or cloud
sources disqualify acceptance.

**CRITICAL TIMING LIMITATION:** coexisting source messages and
receipt-window overlap do NOT verify camera exposure-to-IMU
**physical acquisition time offset**, rolling shutter, true USB
delivery latency, camera–IMU extrinsics, clock absolute phase,
or XRSLAM pose direction. The M18/M19 mapped header stamps are
receipt-anchored. An event-based physical calibration and TF
audit are separate, later milestones. M20 does not operate motors,
activate navigation or authorize collision avoidance.

## Physical safety gate — mandatory

The STM32 USB CDC port is capable of accepting wheel/manipulator
motor-control commands, even though M20 **only** sends the two fixed
IMU stream-configuration requests. Opening the port may reset
the MCU. **Physically remove or inhibit BOTH wheel and manipulator
motor power, engage and verify the physical E-stop, immobilize Leroy,
and keep drive power inhibited until the serial STOP ACK is confirmed
and any MCU reset/stream state has been inspected.** Software
ROS/Podman isolation is not a physical E-stop.

The original motor-capable `leroy-ros` must remain **exited**.
No concurrent MCU tty owner, video owner or alternate ROS bridge.
The complete test uses **one** disposable Podman container with:
`--network none`, private IPC/PID, cap drop, no-new-privileges,
read-only filesystem, tmpfs, only host `/dev/video6` mapped to
`/dev/video2`, and the verified STM32 host `/dev/ttyACM0`
mapped to `/dev/ttyACM0`. No other `/dev/tty*` devices,
motors or host network mapped.

The historically verified STM32 symlink was:
```text
/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00
 -> ../../ttyACM0
```
This is **historical evidence**, not guaranteed for subsequent USB
re-enumeration. Check the live identity again. The front-camera
host `/dev/video6` was verified during M16; check it again and
confirm it still refers to the physical front camera.

## Step 1 — Build/test disposable M20 image (NO device access)

Host commands below run in the **Arch Linux Bash terminal**, not
inside a ROS container. Never source `/opt/ros` on Arch.
Do NOT globally set `set -Eeuo pipefail` in an interactive shell.

```bash
set +e
set +u
set +o pipefail

(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m20-combined-camera-imu-passive
  git pull --ff-only origin feature/m20-combined-camera-imu-passive
  cd ros

  podman image exists localhost/leroy-imu:m19-clock-audit
  podman build --no-cache \
    -f Dockerfile.m20-sensors \
    -t leroy-sensors:m20-camera-imu .
) 2>&1 | tee m20_combined_image_build.log

M20_BUILD_EXIT=${PIPESTATUS[0]}
echo "M20_BUILD_EXIT_CODE=$M20_BUILD_EXIT"
grep -E 'M20_ROS_IMPORT_SELFTEST=PASS|M20_ROS_NODES_SELFTEST=PASS|^OK$' \
  m20_combined_image_build.log || true
```

Require `M20_BUILD_EXIT_CODE=0`, Python unittest `OK`,
`M20_ROS_IMPORT_SELFTEST=PASS` and `M20_ROS_NODES_SELFTEST=PASS`.
No camera, MCU, motor or USB port is accessible during this build.
These tests validate software environment and the *unconnected*
real ROS publisher/observer constructors; they do not validate
physical camera/IMU messages or true synchronization.

## Step 2 — Non-actuating host preflight (do NOT open serial)

**First physically remove/inhibit drive power and engage verified
hardware E-stop.** Compare the real camera to previously validated
front camera, verify the current MCU by-id symlink, inspect serial
ownership, and confirm the original motor-capable container is exited:

```bash
set +e
set +u
set +o pipefail

cd "$(git rev-parse --show-toplevel)/ros"
MCU_BY_ID="/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00"

podman inspect leroy-ros \
  --format 'status={{.State.Status}} image={{.Config.Image}}'
ls -l /dev/video6 /dev/ttyACM0
ls -l /dev/serial/by-id/ 2>/dev/null || true
readlink -f "$MCU_BY_ID"
if command -v fuser >/dev/null 2>&1; then
  fuser -v /dev/ttyACM0 /dev/video6 2>&1 || true
else
  echo "STOP: cannot verify USB ownership without fuser"
fi
```

**STOP** if the normal container is not `exited`, the camera
identity is uncertain, the real by-id symlink does not resolve
to `/dev/ttyACM0`, either device is busy, the offline build
failed, or physical motor isolation is uncertain. Do not bypass
permissions with `--privileged` or extra devices.

## Step 3 — 120-second combined physical sensor observation

Only after all physical/USB preconditions pass. This command
does **not** start the motor-capable original container or ROS
bridge. It writes only IMU stream rate START/STOP commands.

```bash
set +e
set +u
set +o pipefail
cd "$(git rev-parse --show-toplevel)/ros"

MCU_BY_ID="/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00"

if [ "$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null)" != "exited" ]; then
  echo "STOP: original motor-capable leroy-ros must remain exited"
elif [ ! -c /dev/video6 ] || [ ! -c /dev/ttyACM0 ]; then
  echo "STOP: front camera or MCU tty not available"
elif [ "$(readlink -f "$MCU_BY_ID" 2>/dev/null)" != "/dev/ttyACM0" ]; then
  echo "STOP: STM32 MCU USB identity does not match /dev/ttyACM0"
elif ! command -v fuser >/dev/null 2>&1; then
  echo "STOP: cannot check competing device users"
elif fuser /dev/ttyACM0 /dev/video6 >/dev/null 2>&1; then
  echo "STOP: MCU or front camera is used by another process"
elif ! podman image exists localhost/leroy-sensors:m20-camera-imu; then
  echo "STOP: M20 image not built"
else
  read -r -p \
    "Confirmed FRONT camera and STM32, exclusive devices, drive power PHYSICALLY inhibited, verified E-stop, stationary robot? Type ISOLATED: " \
    answer
  if [ "$answer" != "ISOLATED" ]; then
    echo "STOP: physical isolation not confirmed"
  else
    podman run --rm \
      --name leroy-m20-camera-imu-only \
      --network none --ipc private --pid private \
      --shm-size 512m \
      --cap-drop all --security-opt no-new-privileges \
      --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
      --group-add keep-groups \
      --device /dev/video6:/dev/video2 \
      --device /dev/ttyACM0:/dev/ttyACM0 \
      -e LEROY_M20_SENSOR_ONLY=1 \
      -e LEROY_NAV2_DRY_RUN=1 \
      -e ROS_DOMAIN_ID=170 \
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      localhost/leroy-sensors:m20-camera-imu \
      2>&1 | tee m20_real_camera_imu.log

    M20_CAPTURE_EXIT=${PIPESTATUS[0]}
    echo "M20_CAPTURE_EXIT_CODE=$M20_CAPTURE_EXIT"
  fi
fi
```

If interrupted, the runner attempts SIGTERM cleanup for the Python
IMU collector **before** terminating the camera launch.
It also may escalate to SIGKILL after bounded waits, which can
prevent a verified IMU-stream stop. The source itself attempts
`set_imu_stream_hz 0` on shutdown, but no software guarantee
survives abrupt process death, device loss or power interruption.
If the STOP ACK is not **PASS**, keep motor power isolated and
independently inspect/reset the MCU under supervision before
resuming other operations.

Do not start, stop or restart `leroy-ros`. If a disposable container
becomes stuck and absolutely needs stopping from a second terminal:

```bash
podman stop --time 15 leroy-m20-camera-imu-only
```

## Step 4 — Extract reports, provenance and container state

```bash
cd "$(git rev-parse --show-toplevel)/ros"

echo "=== M20 COMBINED CAMERA + IMU CONTRACT ==="
grep -E '^(M20_IMU_STREAM_START_ACK|M20_IMU_STREAM_STOP_ACK|M20_COMBINED_REPORT_JSON|M20_COMBINED_VERDICT|M20_COMBINED_ERROR|M16_VERDICT|M20_SHUTDOWN)=' \
  m20_real_camera_imu.log || true

echo "=== ORIGINAL MOTOR-CAPABLE CONTAINER ==="
podman inspect leroy-ros \
  --format 'status={{.State.Status}} image={{.Config.Image}}'
echo "=== DISPOSABLE CONTAINER ==="
podman ps -a --filter name=leroy-m20-camera-imu-only \
  --format '{{.Names}} {{.Status}}'
```

Only `M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT`,
the two MCU stream ACKs `PASS`, the full 120-second report and
`M20_CAPTURE_EXIT_CODE=0`, with original `leroy-ros` still
`exited`, count as limited **M20 concurrent ROS message contract**.
Do **not** mistake the intentionally incomplete overall M16 status
or experimental IMU publisher provenance warning for M20 failures.

Until separately validated with physically correlated sensor events,
there is **NO** measured absolute camera/IMU offset, calibrated
camera–IMU spatial transform, XRSLAM localization readiness,
odometry or active motor/navigation authorization.

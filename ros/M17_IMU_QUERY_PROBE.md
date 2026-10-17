# M17 — Non-actuating physical MCU/IMU query probe (XRSLAM prerequisites)

**Intent:** follow the M16 real camera/DepthART message-contract success
without optimizing DepthART on CPU, starting `leroy-ros`, or engaging motors.
This is only an **unstamped firmware IMU sampling probe**. It is NOT
`/imu/data_raw` validation, XRSLAM pose qualification or permission to
drive.

## Why NOT use the normal bridge

The existing `leroy_bridge` opens the motor-capable MCU's USB serial port,
subscribes to `/cmd_vel`, sends serial `set_wheels` and `keepalive`
commands, and starts IMU and encoder streaming. The checked-in
`leroy_bridge.yaml` opts into provisional positive PWM gain. A
`wheel_velocity_to_pwm_gain=0` edit in a source checkout cannot change
an already-running bridge. **Do not start that bridge to collect IMU data.**

The STM32 firmware's `command_init` initializes `g_imu_stream_hz=0`.
A pure serial reader may therefore observe no periodic IMU data. The
non-actuating firmware query `get_imu_robot` returns
`imu_robot=ok calibrated=... accel_forward_mps2=... ...` or a named
unavailable/read-error response. The six measurements are signed,
three-decimal SI strings; accelerations are in +X forward, +Y left,
+Z up, while gyro values are roll/pitch/yaw rad/s.

**Crucial distinction:** querying requires writing the exact bytes
`get_imu_robot\r\n` to the USB CDC serial port. Although the firmware
implements it as a read-only query rather than a wheel command, the USB
serial interface is also a *motor-control interface*. Software isolation
alone cannot guarantee immobilization. **Physically inhibit wheel and
manipulator motor power and verify the hardware E-stop before opening the
MCU port.** Opening CDC ACM can reset the MCU or alter serial state.
The query tool sends **no** watchdog, keepalive, stream configuration,
zero-motion or wheel command. Do not run it concurrently with a bridge,
serial console or other MCU controller.

The query **does not return MCU sequence or acquisition timestamp**.
Host poll times cannot be used as qualified IMU acquisition timestamps.
No ROS publisher is created, and this cannot be fed into XRSLAM.

## Code and strict boundaries

- `m17_imu_query_probe.py`: standalone 15-second, 2-Hz fixed-poll
  query (adjustable 5–30-second duration only), fixed **in-container**
  `/dev/ttyACM0`, a single `Serial.write(QUERY)` callsite,
  `QUERY=b"get_imu_robot\r\n"`. Parses exactly six fixed-point
  sensor fields plus calibration flag, explicitly bounds requests,
  validates finite values and reports only summary statistics.
  No raw IMU transcript or identifying USB strings are saved.
- `m17_imu_probe_test.py`: offline protocol and AST tests of the one
  write callsite and exact bytes, field parsing and disallowed forms.
- `Dockerfile.m17-imu`: separate lightweight derived image from the
  existing locally built `leroy-ros:jazzy-lowpwm`. Runs full
  offline parser/write-allowlist tests and a CLI self-test during build.
  The base image still *contains* the old bridge on disk; the M17 image's
  CMD runs ONLY the probe. No ROS/bridge process starts.

The tool prints `M17_IMU_QUERY_REPORT_JSON=...` and
`M17_IMU_QUERY_VERDICT=...`; possible verdicts are
`OBSERVED_UNSTAMPED_IMU_QUERY_SAMPLES` or
`INCOMPLETE_OR_UNVERIFIED_IMU_QUERY`. A positive verdict requires
five valid samples and no malformed protocol responses. The gyro
calibration flag is recorded separately and is NOT a trusted physical
calibration acceptance.

## Step 1 — Checkout and build (NO hardware access)

**All following blocks run on the Arch Linux HOST.** Do not source
`/opt/ros` or `/leroy_ws/install` on the host. Avoid inherited
`set -e` when troubleshooting, so Podman exit doesn't close the shell.

```bash
set +e
set +u
set +o pipefail

cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m17-isolated-nonactuating-imu-query
git pull --ff-only origin feature/m17-isolated-nonactuating-imu-query
cd ros

podman build --no-cache -f Dockerfile.m17-imu \
  -t leroy-imu:m17-query-only . 2>&1 | tee m17_imu_image_build.log
M17_BUILD_EXIT=${PIPESTATUS[0]}
echo "M17_BUILD_EXIT_CODE=$M17_BUILD_EXIT"
grep 'M17_IMU_QUERY_SELFTEST=PASS' m17_imu_image_build.log || true
```

Require `M17_BUILD_EXIT_CODE=0`, all unittest cases to pass, and
`M17_IMU_QUERY_SELFTEST=PASS` before proceeding. This build touches
**no** MCU USB port.

## Step 2 — Operator physical and serial preflight (NO serial command)

**First:** verify wheel/manipulator motor power physically inhibited,
hardware E-stop applied, robot secured against movement, and no
concurrent MCU serial process. If this cannot be established, **stop
here**. This preflight only enumerates interfaces and container status.

```bash
set +e
set +u
set +o pipefail

cd "$(git rev-parse --show-toplevel)/ros"
echo '=== EXISTING MOTOR-CAPABLE CONTAINER ==='
podman inspect leroy-ros --format 'status={{.State.Status}} image={{.Config.Image}}'
echo '=== CONFIGURED MCU SERIAL DEVICE ==='
ls -l /dev/ttyACM0
test -c /dev/ttyACM0
echo "MCU_DEVICE_EXISTS_EXIT=$?"
echo '=== USB SERIAL DEVICE IDENTITIES (IDENTIFICATION ONLY) ==='
if [ -d /dev/serial/by-id ]; then
  ls -l /dev/serial/by-id/
fi
echo '=== OTHER USERS OF MCU SERIAL DEVICE ==='
if command -v fuser >/dev/null 2>&1; then
  fuser -v /dev/ttyACM0 2>&1 || true
else
  echo 'fuser not installed; check serial ownership manually'
fi
```

`/dev/ttyACM0` is the **historical configuration**, not a verified
current host port. Compare by-id identities and hardware knowledge;
if another device is mapped there, STOP. Do NOT map any ambiguous tty.
Likewise STOP if `leroy-ros` is running or any process owns the port.
The check does not remotely attest that physical power is inhibited.

## Step 3 — Bounded non-actuating IMU query

**Run only after the physical and USB identity checks above pass.**
This maps the **one validated MCU serial device** only. No camera,
network/ROS bridge, serial stream configuration, or other host device.
Rootless Podman retains your already-authorized host group access via
`--group-add keep-groups`; do not bypass errors by broadly changing
host device permissions.

```bash
set +e
set +u
set +o pipefail

cd "$(git rev-parse --show-toplevel)/ros"

if [ "$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null)" != "exited" ]; then
  echo "STOP: motor-capable leroy-ros must remain stopped"
elif [ ! -c /dev/ttyACM0 ]; then
  echo "STOP: verified MCU serial device is missing"
else
  podman run --rm \
    --name leroy-m17-imu-query \
    --network none --ipc private --pid private \
    --cap-drop all --security-opt no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=64m \
    --group-add keep-groups \
    --device /dev/ttyACM0:/dev/ttyACM0 \
    leroy-imu:m17-query-only \
    2>&1 | tee m17_actual_imu_query.log
  M17_CAPTURE_EXIT=${PIPESTATUS[0]}
  echo "M17_CAPTURE_EXIT_CODE=$M17_CAPTURE_EXIT"
fi
```

The probe runs ~15 seconds and exits. If the serial device cannot be
opened, capture the actual error; do not start the normal bridge.
No motors should be energized during the test. In a second host terminal,
if absolutely necessary to stop **only** this disposable container:

```bash
podman stop --time 5 leroy-m17-imu-query
```

## Step 4 — Extract results

```bash
cd "$(git rev-parse --show-toplevel)/ros"
grep -E '^M17_(IMU_QUERY_REPORT_JSON|IMU_QUERY_VERDICT|IMU_QUERY_ERROR)=' \
  m17_actual_imu_query.log || true
podman inspect leroy-ros --format 'original_status={{.State.Status}}'
podman ps -a --filter name=leroy-m17-imu-query \
  --format '{{.Names}} {{.Status}}'
```

### Interpretation and XRSLAM readiness

A valid response proves the currently connected MCU firmware could read
an IMU sample in its robot-axis convention at that time. It does NOT
verify sampling at 100 Hz, embedded sequence/timestamp or MCU/host clock
mapping, absolute camera–IMU temporal offset, axis/extrinsic calibration,
verified static TF, pose convention, metric odometry, or stopped motor
hardware. Do not produce a synthetic `/imu/data_raw` from these polls.

**Future stage, gated independently:** design a sensor-only ROS IMU
publisher that explicitly configures **only** IMU streaming, checks
MCU `seq` and `t_us`, uses the existing tested `McuClockSync`
mapping without claiming absolute camera offset, and allows
camera/IMU stream contract observation in an isolated DDS domain.
Such a source must never instantiate `LeroyBridgeNode` or subscribe
to `/cmd_vel`. Do not activate XRSLAM `monocular_imu` or set
`pose_convention_validated=true` merely on these probe results.
The M16 real camera+DepthART message-contract acceptance is preserved;
CPU DepthART optimization is explicitly deferred.

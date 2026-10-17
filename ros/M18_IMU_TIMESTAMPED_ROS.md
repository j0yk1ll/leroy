# M18 — Isolated, MCU-timestamped ROS IMU only; no motor bridge

**Objective:** go beyond M17's 30/30 valid **unstamped** physical MCU
queries and collect **actual ROS `sensor_msgs/msg/Imu`** at
`/imu/data_raw`, with MCU sequence and `t_us` telemetry timing.
This is an isolated, diagnostic software contract check. Previous M16
real camera/DepthART ROS message-contract acceptance is preserved.
**CPU DepthART performance work stays deferred.**

## Safety and scope

The MCU serial device `/dev/ttyACM0` is a **shared motor-control
interface**. M18 is not a hardware safety interlock. The operator must
**physically remove/inhibit wheel and manipulator motor power**, engage
the verified E-stop, and immobilize the robot before allowing Podman to
open the MCU serial device. Opening USB CDC may toggle DTR/restart the
MCU. M17 did not establish physical motor-power isolation from logs;
verify it again before this separate run.

- The M18 program is **not `LeroyBridgeNode`**; it has no subscriber,
  controller, goal/action client, motor ROS topic, encoder publisher,
  TF broadcaster or XRSLAM instance. It publishes only
  `/imu/data_raw` with BEST_EFFORT ROS sensor QoS in **domain 169**
  inside a `--network none` Podman container.
- The **only two allowed MCU serial commands** are
  `set_imu_stream_hz 50\r\n` on startup and
  `set_imu_stream_hz 0\r\n` at cleanup. These configure **IMU telemetry
  only**, not platform or manipulator motion. No wheel, watchdog,
  encoder-rate or keepalive command is sent. Firmware acknowledges
  both via `ok set_imu_stream_hz`.
- An exclusive USB serial open is requested using PySerial
  `exclusive=True`. This coordinates with cooperating processes,
  not a guarantee against every alternate owner. No concurrent
  `leroy-ros` or serial terminal may run.
- **SIGINT/SIGTERM** set a shutdown flag so the source attempts to
  disable IMU streaming in a `finally` block. If the process is
  SIGKILLed or USB fails, it might NOT restore stream rate to zero.
  Keep motor power physically inhibited until MCU state is independently
  verified; do not assume container teardown means MCU telemetry stopped.

The M18 source parses the exact firmware packet:
```text
telemetry imu seq=<uint32> t_us=<uint32> cal=<0|1> af_mm_s2=<int32> al_mm_s2=<int32> au_mm_s2=<int32> gr_mrad_s=<int32> gp_mrad_s=<int32> gy_mrad_s=<int32> mag_valid=<0|1> mx_raw=<int32> my_raw=<int32> mz_raw=<int32>
```
It rejects missing/extra/malformed fields; records MCU sequence gaps,
and latches failures on duplicated/reordered/restarted sequence,
MCU timestamp jumps or host ROS-clock steps. It withholds data marked
gyro-uncalibrated by firmware. `McuClockSync` reuses the tested,
one-way receipt-based clock **rate** estimator. The first stamp is
anchored ~5 ms before host receipt; **that 5 ms is NOT a measured USB
delay**. Absolute MCU-to-camera offset/phase, IMU noise and spatial
extrinsics remain unverified. Firmware `t_us` is a real MCU clock value,
but mapped ROS stamps are estimates, not a verified camera sync.

M18 prints both:
- `M16_PASSIVE_REPORT_JSON=...`, `M16_VERDICT=...` from the existing
  read-only observer, and
- `M18_IMU_STREAM_REPORT_JSON=...`,
  `M18_IMU_STREAM_VERDICT=...`, as well as
  `M18_IMU_STREAM_START_ACK` and `M18_IMU_STREAM_STOP_ACK`.

The deliberate nonproduction publisher name `/m18_imu_source` will
trigger the existing M16 observer's **IMU publisher provenance warning**:
the original integrated contract expects `/leroy_bridge`. This is
**correct** for the isolated M18 experiment and is NOT silently
suppressed. Full M16 verdict remains incomplete because no camera,
depth, XRSLAM, obstacle stream or odometry is launched.

## Step 1 — Build and offline ROS-node smoke; NO serial access

Run the entire block in the **Arch Linux host terminal** from a Leroy
checkout. Do NOT paste any `source /opt/ros` lines on Arch. Do not
enable `set -e` in the interactive shell while debugging.

```bash
set +e
set +u
set +o pipefail

cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m18-passive-imu-clock-audit
git pull --ff-only origin feature/m18-passive-imu-clock-audit
cd ros

podman build --no-cache \
  -f Dockerfile.m18-imu \
  -t leroy-imu:m18-stamped-ros . \
  2>&1 | tee m18_imu_image_build.log

M18_BUILD_EXIT=${PIPESTATUS[0]}
echo "M18_BUILD_EXIT_CODE=$M18_BUILD_EXIT"
grep -E 'M18_ROS_NODES_SELFTEST=PASS|^OK$' \
  m18_imu_image_build.log || true
```

Only proceed if build exit = 0, offline unittest **OK** and
`M18_ROS_NODES_SELFTEST=PASS`. This smoke instantiates the real
M18 publisher and M16 observer **without opening a tty or emitting
real sensor data**. It does not qualify hardware performance or
camera–IMU timing.

## Step 2 — Read-only USB/host preflight

**Physically inhibit wheel and manipulator motor power, engage a
verified E-stop and secure the robot first.** Run the following host
commands and verify that `/dev/ttyACM0` is indeed the known STM32
Leroy MCU, not another USB serial device. Device names may change.
No tty is opened by these commands.

```bash
set +e
set +u
set +o pipefail
cd "$(git rev-parse --show-toplevel)/ros"

echo "=== MOTOR-CAPABLE ORIGINAL CONTAINER ==="
podman inspect leroy-ros \
  --format 'status={{.State.Status}} image={{.Config.Image}}'
echo "=== MCU USB CHARACTER DEVICE ==="
ls -l /dev/ttyACM0
test -c /dev/ttyACM0
echo "MCU_CHARACTER_DEVICE_EXIT=$?"
echo "=== STABLE USB IDENTIFIERS ==="
if [ -d /dev/serial/by-id ]; then
  ls -l /dev/serial/by-id/
fi
echo "=== MCU PORT USERS ==="
if command -v fuser >/dev/null 2>&1; then
  fuser -v /dev/ttyACM0 2>&1 || true
else
  echo 'fuser unavailable: verify all serial-process ownership separately'
fi
```

**STOP** if `leroy-ros` is not `exited`, if device identity is
uncertain, if other processes may hold the port, if power isolation
cannot be established, or if the M18 build failed. Do not resolve
permission errors by using `--privileged`, mapping extra ports, or
modifying MCU firmware.

## Step 3 — Physical-power-gated, isolated ROS IMU acquisition

Only after the physical and device checks above pass:

```bash
set +e
set +u
set +o pipefail
cd "$(git rev-parse --show-toplevel)/ros"

if [ "$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null)" != "exited" ]; then
  echo "STOP: original motor-capable leroy-ros is not stopped"
elif [ ! -c /dev/ttyACM0 ]; then
  echo "STOP: MCU tty missing or changed"
elif ! podman image exists localhost/leroy-imu:m18-stamped-ros; then
  echo "STOP: M18 image not built"
else
  read -r -p \
    "MCU identity and exclusive ownership checked; motor power physically inhibited and E-stop engaged? Type ISOLATED: " \
    confirmation
  if [ "$confirmation" != "ISOLATED" ]; then
    echo "STOP: physical isolation not confirmed"
  else
    podman run --rm \
      --name leroy-m18-imu-only \
      --network none \
      --ipc private --pid private \
      --cap-drop all --security-opt no-new-privileges \
      --read-only \
      --tmpfs /tmp:rw,nosuid,nodev,size=128m \
      --group-add keep-groups \
      --device /dev/ttyACM0:/dev/ttyACM0 \
      -e ROS_DOMAIN_ID=169 \
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      -e LEROY_M18_IMU_STREAM=1 \
      -e LEROY_NAV2_DRY_RUN=1 \
      leroy-imu:m18-stamped-ros \
      2>&1 | tee m18_real_stamped_imu.log

    M18_CAPTURE_EXIT=${PIPESTATUS[0]}
    echo "M18_CAPTURE_EXIT_CODE=$M18_CAPTURE_EXIT"
  fi
fi
```

The diagnostic runs for **30 seconds** and attempts to disable MCU
IMU streaming before exiting. If interrupted, Ctrl+C should cause
bounded cleanup; in a second host terminal, if it remains stuck:

```bash
podman stop --time 10 leroy-m18-imu-only
```

**Do not** use `podman stop/ start leroy-ros`. If SIGKILL/USB failure
prevents confirming `M18_IMU_STREAM_STOP_ACK=PASS`, keep the motor
power inhibited and inspect/reset MCU state under separate supervision
before reconnecting hardware.

## Step 4 — Extract ROS sample provenance and cleanup

```bash
cd "$(git rev-parse --show-toplevel)/ros"
echo "=== M18 VERDICTS / ACKS ==="
grep -E '^(M16_PASSIVE_REPORT_JSON|M16_VERDICT|M18_IMU_STREAM_REPORT_JSON|M18_IMU_STREAM_VERDICT|M18_IMU_STREAM_START_ACK|M18_IMU_STREAM_STOP_ACK|M18_IMU_STREAM_ERROR)=' \
  m18_real_stamped_imu.log || true
echo "=== ORIGINAL MOTOR-CAPABLE CONTAINER ==="
podman inspect leroy-ros \
  --format 'status={{.State.Status}} image={{.Config.Image}}'
echo "=== DISPOSABLE IMU CONTAINER ==="
podman ps -a --filter name=leroy-m18-imu-only \
  --format '{{.Names}} {{.Status}}'
```

### Acceptance criteria

`M18_IMU_STREAM_VERDICT=OBSERVED_STAMPED_IMU_ROS_MESSAGE_CONTRACT`
requires fixed firmware start and stop **ACKs**, at least 100
published messages, at least 50 accepted by the M16 observer,
correct ROS message shape/frame/time freshness, exactly one
`/m18_imu_source` publisher, no malformed packets, no MCU sequence
gaps, no timestamp or sequence faults, no uncalibrated packets, and
a fresh final receipt. No raw sample is stored in the report.
In case of low actual stream rate, errors or serial loss, preserve
the **incomplete/unverified** verdict and examine the counters.

The overall M16 verdict remains incomplete by design. **Passing M18
does not validate** absolute camera–IMU timing, real calibrated IMU
extrinsics/noise, XRSLAM `pose_convention_validated`, odom/TF,
obstacle projections, Nav2 use or real-world motor safety. Do not
start XRSLAM, enable `/cmd_vel` or re-enable motor power as part of
this experiment.

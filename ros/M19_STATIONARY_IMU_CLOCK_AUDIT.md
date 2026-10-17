# M19 — Stationary 120-second MCU clock-rate audit (isolated IMU only)

## Purpose and prior evidence

M18 actual MCU/ROS IMU stream **passed**: 1500/1500 accepted
`/imu/data_raw` messages in 30 seconds at 50 Hz, 0 sequence gaps,
0 timestamp or parser faults, and start/stop IMU-stream ACKs.
M18's `clock_rate_estimate_ready=false` was **expected**:
`McuClockSync` needs at least 45 seconds of MCU elapsed time, plus
multiple independent 5-second windows, to fit the MCU/host clock rate.
The 0.0 ppm display from M18 was a default, **not** an observed
zero-drift estimate.

M19 runs the same fixed IMU-only source for **120 seconds**, enough
for the existing estimator's rate fit to become ready and remain
observable. Its acceptance gate additionally requires
`clock_rate_estimate_ready=true`, a finite estimate within the clock
mapper's existing ±200 ppm fault envelope, >5700 actual published
IMU samples, >5600 passively accepted ROS messages, start and stop
firmware acknowledgements and no sequence, payload or clock faults.

This experiment **does not** change firmware, the M18 parser, clock
mapping, motor bridge, camera or DepthART code, ROS message frames,
publisher roles, or safety limits. It does not launch XRSLAM or Nav2.
The normal motor-capable `leroy-ros` remains stopped.

The M19 image extends the **locally built and tested**
`localhost/leroy-imu:m18-stamped-ros` image; its exact ROS publisher
is still `/m18_imu_source` on `/imu/data_raw` in DDS domain 169,
not `/leroy_bridge`. The existing M16 passive observer intentionally
reports this distinct provenance and missing non-IMU streams as an
incomplete overall M16 verdict. **Neither finding is an M19 defect.**

### Physical and timing boundaries

MCU USB CDC is shared with the motors. M19's only possible outgoing
serial writes are `set_imu_stream_hz 50\r\n` and
`set_imu_stream_hz 0\r\n` during cleanup. They control **sensor
telemetry rate only**, but opening the device itself can reset the
MCU. `exclusive=True` is not an interlock. **Before opening the MCU
port, physically disconnect/inhibit wheel and manipulator motor power,
engage a verified E-stop and immobilize Leroy.** Inspect the port
identity and exclusive ownership again, even though M18 passed.
Do not run the normal `leroy-ros` container, bridge, or a concurrent
serial client.

The mapper uses one-way USB delivery observations to estimate relative
MCU vs host **clock rate**, not absolute phase. `ready=true` after
120 seconds does **not** certify the camera–IMU physical time offset,
sensor extrinsics, VIO noise, metric odometry, XRSLAM state, motor stop
performance or obstacle safety. Any physical event-based synchronization
must be independently validated after this test. CPU DepthART
performance investigation remains deferred by operator decision.

## M19 build regression — ROS Jazzy Python path

The initial M19 image build failed at step 6 when importing the unit test:
```text
ModuleNotFoundError: No module named 'rclpy'
```
The Dockerfile sourced the ROS setup scripts, then replaced the required
ROS/ament Python paths with `PYTHONPATH=/opt/m19:/opt/m18:/opt/m18/monitor`.
The corrected image layer **retains** those paths by exporting
`PYTHONPATH="/opt/m19:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"`.
A required build-time import check now confirms `rclpy`, the installed
clock mapper and M19 module can all load, printing
`M19_ROS_PYTHONPATH_SELFTEST=PASS`. This check runs without hardware.

## Step 1 — Build M19 (host Arch shell, NO USB access)

No `source /opt/ros` on Arch Linux. Do not enable host interactive
`set -e`; a failed Podman command should not close the terminal.
M18 image must still exist locally (the previous validated build).

```bash
set +e
set +u
set +o pipefail

(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch fix/m19-ros-pythonpath-image-build
  git pull --ff-only origin fix/m19-ros-pythonpath-image-build
  cd ros

  podman image exists localhost/leroy-imu:m18-stamped-ros
  podman build --no-cache \
    -f Dockerfile.m19-imu \
    -t leroy-imu:m19-clock-audit .
) 2>&1 | tee m19_imu_clock_image_build.log

M19_BUILD_EXIT=${PIPESTATUS[0]}
echo "M19_BUILD_EXIT_CODE=$M19_BUILD_EXIT"
grep -E 'M19_ROS_PYTHONPATH_SELFTEST=PASS|M19_ROS_CLOCK_AUDIT_SELFTEST=PASS|^OK$' \
  m19_imu_clock_image_build.log || true
```

**Required:** `M19_BUILD_EXIT_CODE=0`, Python unittest `OK`,
`M19_ROS_PYTHONPATH_SELFTEST=PASS` and
`M19_ROS_CLOCK_AUDIT_SELFTEST=PASS`. This uses **no** USB device,
physical camera, motor bridge or long-duration clock estimator.
It verifies software policy and constructs actual ROS nodes only.

## Step 2 — Stationary physical/USB preflight (no serial open)

**First** physically disable both wheel and manipulator drive power,
verify the real hardware E-stop is engaged, and make sure Leroy is
immobilized. If hardware isolation is not possible, **STOP**.

```bash
set +e
set +u
set +o pipefail
cd "$(git rev-parse --show-toplevel)/ros"

echo '=== MOTOR-CAPABLE CONTAINER ==='
podman inspect leroy-ros \
  --format 'status={{.State.Status}} image={{.Config.Image}}'

echo '=== MCU CHARACTER DEVICE ==='
ls -l /dev/ttyACM0
test -c /dev/ttyACM0
echo "M19_MCU_TTY_EXISTS_EXIT=$?"

echo '=== STABLE USB DEVICE IDENTIFIERS ==='
if [ -d /dev/serial/by-id ]; then
  ls -l /dev/serial/by-id/
fi

echo '=== MCU SERIAL OWNERS ==='
if command -v fuser >/dev/null 2>&1; then
  fuser -v /dev/ttyACM0 2>&1 || true
else
  echo 'fuser unavailable; verify serial ownership by other means'
fi
```

`/dev/ttyACM0` is the historically configured STM32 tty, not an
identity guarantee. Match the device to the previously verified
MCU /dev/serial/by-id entry before continuing. STOP on missing,
ambiguous, changed or busy tty, or if original `leroy-ros` isn't
`exited`. Do not fix device access with `--privileged` or map
additional USB/serial devices.

## Step 3 — Bounded 120-second stationary IMU audit

Only after the image smoke passes AND physical/device checks above
are independently satisfied. This runs the **separate** M19 image,
with no host network, no camera device and only one MCU USB tty:

```bash
set +e
set +u
set +o pipefail

cd "$(git rev-parse --show-toplevel)/ros"

if [ "$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null)" != "exited" ]; then
  echo 'STOP: motor-capable leroy-ros must stay exited'
elif [ ! -c /dev/ttyACM0 ]; then
  echo 'STOP: MCU serial port missing'
elif ! podman image exists localhost/leroy-imu:m19-clock-audit; then
  echo 'STOP: M19 image unavailable'
else
  read -r -p \
    'Confirm MCU identity, port unowned, wheel/manipulator drive power physically inhibited and E-stop verified. Type ISOLATED: ' \
    confirmation
  if [ "$confirmation" != "ISOLATED" ]; then
    echo 'STOP: physical safety preconditions not confirmed'
  else
    podman run --rm \
      --name leroy-m19-imu-clock-only \
      --network none --ipc private --pid private \
      --cap-drop all --security-opt no-new-privileges \
      --read-only --tmpfs /tmp:rw,nosuid,nodev,size=128m \
      --group-add keep-groups \
      --device /dev/ttyACM0:/dev/ttyACM0 \
      -e LEROY_M18_IMU_STREAM=1 \
      -e LEROY_M19_CLOCK_AUDIT=1 \
      -e LEROY_NAV2_DRY_RUN=1 \
      -e ROS_DOMAIN_ID=169 \
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      leroy-imu:m19-clock-audit \
      2>&1 | tee m19_real_imu_clock.log

    M19_CAPTURE_EXIT=${PIPESTATUS[0]}
    echo "M19_CAPTURE_EXIT_CODE=$M19_CAPTURE_EXIT"
  fi
fi
```

M19 sends a fixed IMU-stream start command, observes 120 seconds
of MCU timestamps and ROS IMU receipts, and attempts the fixed
stream-stop command on exit. If user interrupts with Ctrl+C, the
report must remain **incomplete**, not a pass. If absolutely required,
in another host terminal stop ONLY the disposable M19 container:

```bash
podman stop --time 10 leroy-m19-imu-clock-only
```

If `M19_IMU_STREAM_STOP_ACK` is `UNVERIFIED` (e.g. SIGKILL,
serial disconnect or error), do NOT assume the MCU reverted to
zero IMU stream rate. Maintain physical motor power isolation until
its state is independently inspected by an operator.

## Step 4 — Extract clock results

```bash
cd "$(git rev-parse --show-toplevel)/ros"

echo '=== M19 ACKS, CLOCK AUDIT AND VERDICT ==='
grep -E '^(M19_IMU_STREAM_START_ACK|M19_IMU_STREAM_STOP_ACK|M19_IMU_CLOCK_AUDIT_JSON|M19_IMU_CLOCK_VERDICT|M19_IMU_CLOCK_ERROR|M16_VERDICT)=' \
  m19_real_imu_clock.log || true

echo '=== ORIGINAL MOTOR-CAPABLE CONTAINER ==='
podman inspect leroy-ros \
  --format 'status={{.State.Status}} image={{.Config.Image}}'
echo '=== M19 DISPOSABLE CONTAINER ==='
podman ps -a --filter name=leroy-m19-imu-clock-only \
  --format '{{.Names}} {{.Status}}'
```

**Accept only** `M19_IMU_CLOCK_VERDICT=OBSERVED_STATIONARY_IMU_CLOCK_RATE_CONTRACT`,
both firmware ACKs `PASS`, no fault counters, and capture exit 0.
Do not read `clock_rate_estimate_ppm=0.0` as validated unless
`clock_rate_estimate_ready=true`. Any accepted rate estimate is
**host-relative** and does not establish physical camera phase.

No camera/DepthART test, no XRSLAM launch, odometry, TF, Nav2
controller, wheel, manipulator or firmware change is part of M19.

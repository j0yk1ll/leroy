# M22F — Actual ROS bridge lifecycle with injected FAKE CDC

## Verified baseline

M22E passed **52/52** offline tests on the operator's Arch host:
23 signed-session tests, 14 M22D real-writer tests, and 15 M22C
fake-sink regressions. PR #84 merged into `main` at
`883effc24b8ec1890135d33ff5a7812065f5b7e5`.

## Scope of this milestone

M22F instantiates the ACTUAL current `LeroyBridgeNode` in ROS 2 Jazzy,
inside a disconnected rootless Podman runtime. **BEFORE constructing
the bridge**, its module's `serial.Serial` factory is replaced with
a strict in-memory-only `MemoryCDC` fake accepting only
`m22f://memory`; no OS serial constructor is ever called.

All ROS parameter overrides configure this fake URI, disable IMU and
encoder streaming, and preserve the positive 500ms watchdog.
The fake CDC parser insists every outgoing packet independently
matches the existing M22D `command_to_bytes()` allowlist. The overlay
image copies the exact current bridge package source into `/opt/m22f`
so the test does not silently import an older inherited build.

The test exercises:

- Exact production bridge constructor and fake serial startup
  (`set_watchdog_ms 500`, both telemetry rates zero)
- Actual ROS 2 executor and 10Hz keepalive timer
- Repeated NONZERO ROS Twist injection on `/cmd_vel` from a fake
  publisher while the bridge has **no matching subscription**
- Direct attempt to call actual `_send_command(nonzero set_wheels)`;
  verify false return and no fake serial write
- Canonical zero-wheel CDC packet allowed
- Clock-fault keepalive withholding
- Injected fake CDC write exception is surfaced as failure
- Actual ROS node `destroy_node()` sends zero and disables streams,
  closes fake serial, and joins the fake reader thread
- After the run, require zero nonzero wheel packets and no physical
  stop or ACK claims

The current bridge's transport write exceptions are **reported**,
but its command writer does NOT yet terminal-latch all transport
failures. M22F documents this gap explicitly. No signed M22E motion
protocol is connected to the bridge in this milestone; **real
nonzero motion remains disabled by the M22D allowlist**.

## Build — rootless Arch Podman, no network

Requires previously validated local base image
`localhost/leroy-sensors:m22b-authority-selftest`.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22f-no-device-bridge-lifecycle
  git pull --ff-only origin feature/m22f-no-device-bridge-lifecycle
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m22b-authority-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m22f-selftest \
    -t leroy-sensors:m22f-fake-bridge-lifecycle .
) 2>&1 | tee m22f_no_device_build.log
M22F_BUILD_EXIT=${PIPESTATUS[0]}
echo "M22F_BUILD_EXIT_CODE=$M22F_BUILD_EXIT"
```

Require `M22F_REAL_BRIDGE_IMPORT_SELFTEST=PASS` and build exit 0.

## Run — no USB, camera, motor, or volume maps

Only after build succeeds:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m22f-no-device-bridge \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m22f-fake-bridge-lifecycle
) 2>&1 | tee m22f_no_device_simulation.log
M22F_SIM_EXIT=${PIPESTATUS[0]}
echo "M22F_SIM_EXIT_CODE=$M22F_SIM_EXIT"
```

Require the following markers and zero exit:

```text
M22F_FAKE_BRIDGE_STARTUP=PASS
M22F_RAW_ROS_BYPASS_INHIBIT=PASS
M22F_CDC_ZERO_WHEELS_ONLY=PASS
M22F_CLOCK_FAULT_KEEPALIVE_INHIBIT=PASS
M22F_FAKE_WRITE_ERROR_DETECTED=PASS
M22F_BRIDGE_STOP_ON_SHUTDOWN=PASS
M22F_ROS_BRIDGE_SELFTEST=PASS
M22F_VERDICT=REAL_ROS_NODE_FAKE_CDC_ZERO_ONLY
M22F_DEVICE_ACCESS=NONE
M22F_SIM_EXIT_CODE=0
```

Do not run `ros2 launch leroy_bringup robot.launch.py`, the
production `leroy-ros` service, the real `leroy_bridge_node`
executable or any physical MCU serial port. **Opening the physical
STM32 CDC port can energize wheels/manipulator regardless of the
zero-only software command guard.** No physical test is approved.

## Engineering work still required

1. Protected command and operator key provisioning; verified ROS
   source binding and host-monotonic timing in independent processes.
2. Bind signed M22E command ingress inside the actual bridge with
   a single-writer transport arbiter. Add protected rearm workflow.
3. Terminal-latch transport/reader faults, audit keepalive/STOP
   behavior, restart session invalidation and process supervision
   with FAKE MCU, plus failure injection.
4. Only after the software-only path passes all of those reviews
   may live sensor integration and later supervised motion validation
   be planned. Hardware-based protective engineering remains
   separately scoped.

# M22G — Terminal transport fault latch in the actual bridge

## Verified baseline

M22F actual Jazzy `LeroyBridgeNode` on injected in-memory CDC passed
all host acceptance markers: 76 unauthorized `/cmd_vel` samples,
14 fake serial packets, **zero nonzero wheel commands**, explicit
fake write failure detection, normal shutdown,
`M22F_DEVICE_ACCESS=NONE`, `M22F_SIM_EXIT_CODE=0`.
PR #85 is merged into `main` at
`19f17b7f92037bfb0b29d97d26ba61643df6e4e4`.

M22F exposed an explicit gap:
`transport_failure_terminal_latch_validated=false`.
M22G closes that particular **software** gap while keeping all
motor-capable ingress disabled by the existing M22D CDC allowlist.

## Production code changes

`ros/src/leroy_bridge/leroy_bridge/node.py` now creates a
per-node `_transport_fault` Event before attempting serial startup.
The actual bridge's sole `_send_command` serial-writer method:

- Continues to check the M22D CDC opcode allowlist BEFORE writing
- Rejects *all* subsequent writes once `_transport_fault` is set
- Checks the terminal fault again under `_serial_lock`
- Treats serial write exceptions, flush exceptions, OSError, and
  short writes as terminal faults
- Sets `_transport_fault` and `_stop_event` while holding the
  serial writer lock; no further keepalive can be delivered
- Does not silently clear or reset the terminal fault event

The telemetry reader now marks read failures terminal and serializes
its fault transition through that same `_serial_lock`.
An expected reader exit during shutdown is not classified as a fault.
The keepalive timer returns without writing if either clock fault
or transport fault is set.

**Terminal fault semantics:** after a fault, even `set_wheels 0 0 0 0`
and telemetry shutdown writes are withheld. A failed/short serial
write does NOT prove STOP delivery; further writes on an unreliable
connection could hide failure. The bridge instead withholds keepalive,
quiesces the reader, and remains latched until reconstructed.
Physical stopping, watchdog timing and MCU ACK are not validated.
The production bridge has NO nonzero wheel ingress, ROS `/cmd_vel`
subscriber, session key provisioning, or M22E signed motion binding.

## Offline test: 10 new + 14 existing M22D regressions

`test_m22g_transport_fault.py` extracts the production
`_send_command` and `_reader_loop` method AST into an in-memory fake
bridge without importing rclpy or creating a serial port. Tests
cover initial health, write/flush/short-write faults, terminal refusal
of keepalive/STOP/streams/config after fault, rejected nonzero motion
without serial side effect, reader disconnect, normal shutdown
reader exit, fault persistence until a freshly constructed bridge,
and static evidence for one guarded serial writer.

Previous M22D fixture now provides the new events. Existing M22D
test expectations remain otherwise unchanged. The M22F ROS regression
is revised: after an injected serial error, it requires an irreversible
fault in that bridge instance and NO further CDC bytes on shutdown.
It then creates a fresh fault-free fake bridge to independently verify
normal shutdown's zero wheel packet and streams-off commands.

## Arch host offline check — NEVER open physical CDC

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22g-terminal-bridge-transport-fault
  git pull --ff-only origin feature/m22g-terminal-bridge-transport-fault
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  export PYTHONPATH="$PWD:$PWD/src/leroy_bridge"
  python3 -u src/leroy_bridge/test/test_m22g_transport_fault.py
  python3 -u src/leroy_bridge/test/test_m22d_command_ingress.py
  python3 -u m22e_signed_session_test.py
  python3 -u m22c_fake_serial_boundary_test.py
  python3 -m pytest -q \
    src/leroy_bridge/test/test_xrslam_contract.py \
    src/leroy_bridge/test/test_motor_pwm.py \
    src/leroy_bridge/test/test_clock_sync.py
) 2>&1 | tee m22g_transport_fault_offline.log
M22G_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22G_OFFLINE_EXIT_CODE=$M22G_OFFLINE_EXIT"
```

Require 10/10 new M22G PASS, M22D 14/14 PASS, M22E 23/23 PASS,
M22C 15/15 PASS, existing bridge pytest 20/20 PASS,
`M22G_TERMINAL_TRANSPORT_FAULT_SELFTEST=PASS`,
`M22G_NONZERO_PRODUCTION_CDC_ENABLED=FALSE`,
`M22G_DEVICE_ACCESS=NONE`, `M22G_OFFLINE_EXIT_CODE=0`.

## Build and run isolated ROS lifecycle using updated M22F harness

Requires local
`localhost/leroy-sensors:m22b-authority-selftest` image.
**No production container launch, no volumes/devices or host network.**

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m22b-authority-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m22f-selftest \
    -t leroy-sensors:m22g-terminal-fault-selftest .
) 2>&1 | tee m22g_no_device_build.log
M22G_BUILD_EXIT=${PIPESTATUS[0]}
echo "M22G_BUILD_EXIT_CODE=$M22G_BUILD_EXIT"
```

Require `M22F_REAL_BRIDGE_IMPORT_SELFTEST=PASS` and build exit 0.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m22g-terminal-fault-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m22g-terminal-fault-selftest
) 2>&1 | tee m22g_no_device_simulation.log
M22G_SIM_EXIT=${PIPESTATUS[0]}
echo "M22G_SIM_EXIT_CODE=$M22G_SIM_EXIT"
```

Require markers:
`M22F_FAKE_BRIDGE_STARTUP=PASS`,
`M22F_RAW_ROS_BYPASS_INHIBIT=PASS`,
`M22F_CDC_ZERO_WHEELS_ONLY=PASS`,
`M22F_CLOCK_FAULT_KEEPALIVE_INHIBIT=PASS`,
`M22F_FAKE_WRITE_ERROR_DETECTED=PASS`,
`M22F_TERMINAL_TRANSPORT_FAULT=PASS`,
`M22G_SHUTDOWN_NO_CDC_AFTER_FAULT=PASS`,
`M22G_FRESH_BRIDGE_SHUTDOWN_ZERO=PASS`,
`M22F_ROS_BRIDGE_SELFTEST=PASS`,
`M22F_DEVICE_ACCESS=NONE`, and `M22G_SIM_EXIT_CODE=0`.

## Important boundaries and next development step

Even a zero-only source bridge may create physical motion upon
opening the MCU CDC port because of USB/DTR or firmware behavior.
Neither this test nor an earlier passive observation authorizes
physical access. Keep the production `leroy-ros` service stopped.
Hardware protective mechanisms remain outside the software project
scope but are separately necessary before physical motion testing.

Next software work should bind the M22E signed command/session/rearm
protocol to actual ROS ingress, provision/protect credentials, and
verify a single non-bypassable real-bridge command entry point in a
completely simulated transport; keep production nonzero actuation
disabled until that integration and its fault matrix are validated.

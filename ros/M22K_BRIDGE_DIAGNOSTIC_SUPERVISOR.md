# M22K — Bridge-owned signed diagnostic supervisor and local fault veto

## Validated baseline

M22J operator Arch host: **74/74 offline PASS** and actual Jazzy
`LeroyBridgeNode` with injected `m22f://memory` fake CDC:
four signed audit checks, distinct M22I signer processes, replay
and session rotation rejection, seven fake CDC writes,
**zero nonzero wheel writes**, clean zero-wheel shutdown,
`M22J_DEVICE_ACCESS=NONE`, `M22J_SIM_EXIT_CODE=0`.
PR #90 merged into `main` at
`68f6b27795559376a936dcb70af015d2dcf87f3c`.

## Objective

M22J's bridge-owned audit verifies individual HMAC signatures but
does not check continuous sensor freshness, explicit rearm dwell,
concurrent source labels or the bridge's OWN transport/clock/stop
flags as one decision. M22K adds a stateful diagnostic supervisor
to the actual bridge **without installing any real motion ingress**.

## Source changes

- `src/leroy_bridge/leroy_bridge/bridge_diagnostic_supervisor.py`
  (new, ROS-free): `BridgeDiagnosticSupervisor` consumes each
  synthetic signed M22E motion candidate via the M22J audit.
  It accepts separately signed rearm only as an explicit rising-edge
  request for a **one-second uninterrupted healthy dwell**.
  Sequence/HMAC/monotonic deadline/replay are inherited from
  M22E; the supervisor additionally requires health flags all
  boolean True, exactly one ADVISORY synthetic owner tuple,
  provisional diagnostic vx/vy <=0.15 and wz <=0.30, and at most
  200ms between armed/rearming evaluations. Any loss or invalid
  request latches state `LATCHED`. Healthy data returning does
  NOT restore diagnostic permission without another signed
  rearm event and full dwell.
- `src/leroy_bridge/leroy_bridge/node.py` (modified):
  `_supervise_signed_diagnostic(...)` accepts an explicitly injected
  exact supervisor instance and calls it with the bridge's own
  `_transport_fault`, `_clock_fault` and `_stop_event` flags.
  Thus callers cannot falsely claim those LOCAL failure states
  are healthy. The result returns only state, reason and
  `diagnostic_permission`; its immutable
  `physical_motion_authorized` is **always False**.
- `src/leroy_bridge/test/test_m22k_bridge_diagnostic_supervisor.py`
  adds **23 pure-Python tests**, compiling and running the
  exact production node method AST against a poisoned serial
  writer. It checks rearm/dwell, fresh proposal consumption,
  replay, old sessions, forged signatures, sensor/source loss,
  over-limit signed values, evaluator stalls, all three LOCAL
  bridge fault flags, recovery and non-actuation invariants.
- `m22k_real_bridge_supervisor_ros_selftest.py` instantiates
  the actual Jazzy bridge using injected M22F `MemoryCDC` fake
  and M22I's separately spawned synthetic motion/rearm signers.
  It verifies bridge-owned signatures, one-second synthetic
  dwell, zero CDC writes during supervision, health and source
  latching, no automatic recovery, actual bridge clock-fault
  veto, injected terminal fake CDC error, no post-fault CDC
  writes and safe fake transport shutdown.
- `Dockerfile.m22k-selftest`, `m22k_no_device_run.sh` package
  and launch ONLY the isolated no-device simulation.

## Critical trust and safety limitations

**Diagnostic permission is not actuator permission.** A valid
signed request may produce `diagnostic_permission=True` only;
it NEVER contains wheel duties, commands, serialized CDC
packets or `physical_motion_authorized=True`.

Neither an authenticated operator nor genuine sensor or ROS
publisher identity is established. M22I's test supervisor has
both keys, signers run as the same OS user, and advisory source
tuples/synthetic sensor booleans are test-controlled. Host
monotonic times are test-provided. These tests do not prove
concurrent multithreaded authority safety, deployment key
custody, trusted sensor acquisition, MCU ACK, or actual stopping.

**No new ROS motion subscriber and no changes to the M22D CDC
allowlist or motor firmware.** The only production serial writer
still refuses every nonzero `set_wheels` command and M22G
continues to latch transport failures. In production no verifier
or diagnostic supervisor is instantiated by normal bridge
startup, and no signed ROS topic is accepted.

Opening real STM32 CDC can energize wheels/manipulator even if
the bridge never sends nonzero PWM. Do NOT start production
`leroy-ros`, launch `robot.launch.py`, open `/dev/ttyACM*` or
expose camera devices. No physical approval has been issued.

## Run offline — Arch, no devices / ROS

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22k-bridge-diagnostic-state-supervisor
  git pull --ff-only origin feature/m22k-bridge-diagnostic-state-supervisor
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  export PYTHONPATH="$PWD:$PWD/src/leroy_bridge:$PWD/src/leroy_bridge/test"
  python3 -u src/leroy_bridge/test/test_m22k_bridge_diagnostic_supervisor.py
  python3 -u src/leroy_bridge/test/test_m22j_bridge_signed_audit.py
  python3 -u m22i_isolated_role_signing_test.py
  python3 -u src/leroy_bridge/test/test_m22g_transport_fault.py
  python3 -u src/leroy_bridge/test/test_m22d_command_ingress.py
  python3 -m pytest -q \
    src/leroy_bridge/test/test_xrslam_contract.py \
    src/leroy_bridge/test/test_motor_pwm.py \
    src/leroy_bridge/test/test_clock_sync.py
) 2>&1 | tee m22k_bridge_supervisor_offline.log
M22K_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22K_OFFLINE_EXIT_CODE=$M22K_OFFLINE_EXIT"
```

Acceptance: M22K **23/23**, M22J 14/14, M22I 16/16,
M22G 10/10, M22D 14/14, existing bridge pytest 20/20
(**97/97** total) PASS;
`M22K_BRIDGE_DIAGNOSTIC_SUPERVISOR_SELFTEST=PASS`,
`M22K_PRODUCTION_NONZERO_CDC_ENABLED=FALSE`,
`M22K_OPERATOR_AUTHENTICATION_IMPLEMENTED=FALSE`,
`M22K_DEVICE_ACCESS=NONE`, `M22K_OFFLINE_EXIT_CODE=0`.

## Build ROS 2 Jazzy fake CDC container

Requires operator's existing local base
`localhost/leroy-sensors:m22j-bridge-signed-audit`.

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m22j-bridge-signed-audit
  podman build --network none --no-cache \
    -f Dockerfile.m22k-selftest \
    -t leroy-sensors:m22k-bridge-supervisor .
) 2>&1 | tee m22k_no_device_build.log
M22K_BUILD_EXIT=${PIPESTATUS[0]}
echo "M22K_BUILD_EXIT_CODE=$M22K_BUILD_EXIT"
```

Require `M22K_BRIDGE_SUPERVISOR_IMPORT_SELFTEST=PASS` and
`M22K_BUILD_EXIT_CODE=0`.

## Run isolated ROS simulation, fake CDC ONLY

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m22k-supervisor-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m22k-bridge-supervisor
) 2>&1 | tee m22k_no_device_simulation.log
M22K_SIM_EXIT=${PIPESTATUS[0]}
echo "M22K_SIM_EXIT_CODE=$M22K_SIM_EXIT"
```

Required acceptance markers:

```text
M22K_REAL_BRIDGE_DIAGNOSTIC_STARTUP_LATCH=PASS
M22K_SIGNED_REARM_DWELL_DIAGNOSTIC_ONLY=PASS
M22K_SUPERVISOR_NO_CDC_IO=PASS
M22K_SENSOR_FAULT_NO_AUTOMATIC_REARM=PASS
M22K_DUPLICATE_SOURCE_LABEL_INHIBIT=PASS
M22K_LOCAL_BRIDGE_CLOCK_FAULT_VETO=PASS
M22K_LOCAL_BRIDGE_TRANSPORT_FAULT_VETO=PASS
M22K_M22G_TERMINAL_CDC_LATCH=PASS
M22K_POST_FAULT_SHUTDOWN_SILENT=PASS
M22K_REAL_BRIDGE_SUPERVISOR_SELFTEST=PASS
M22K_VERDICT=SIGNED_DIAGNOSTIC_VETO_WITH_FAKE_CDC_ONLY
M22K_DEVICE_ACCESS=NONE
M22K_SIM_EXIT_CODE=0
```

## Next software work

Develop a defensible operator-identity and key-provisioning trust
boundary; verify authenticated, non-bypassable ROS source selection
and sensor provenance; then design one supervising bridge ingress
under fake MCU and process crash/replay tests. Continue to deny
all production nonzero motor commands until independent software
release review and a later approved physical test plan.

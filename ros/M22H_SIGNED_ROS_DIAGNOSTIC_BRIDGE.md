# M22H — Signed ROS diagnostics beside actual zero-only bridge

## Verified baseline

M22G **fully accepted** from operator Arch host: 82/82 offline tests
(M22G 10, M22D 14, M22E 23, M22C 15, legacy pytest 20).
The isolated actual ROS bridge lifecycle passed:
terminal serial fault latched, no CDC writes after fault, normal
zero-wheel shutdown on fresh instance, 76 raw `/cmd_vel` injection
samples rejected, zero nonzero wheel writes, no physical device access,
`M22G_SIM_EXIT_CODE=0`. GitHub confirms #86 and #87 merged.
Main baseline: `c170dff2c996c486b075d954128276e86bed307b`.

## Engineering objective

M22E validated signed command/session/rearm logic with a pure-Python
fake serial receiver. M22G validated actual ROS `LeroyBridgeNode`
lifecycle with fake CDC and terminal transport fault handling.
**M22H connects these tests at the ROS transport level**:
a signed-ROS test sidecar shares the SAME isolated ROS executor
with the real bridge instance. It verifies actual DDS transport,
fake signed command/rearm receipt, synthetic health freshness,
fail-closed gate behavior, diagnostic Twist publication and
the production bridge's continued prohibition of nonzero CDC.

## Message flows

Fake `controller_server` publishes HMAC-signed M22E JSON command
envelopes on `/leroy_nav2/diagnostics/m22h/signed_motion`.
Fake operator publishes independently HMAC-signed one-shot rearm
envelopes on `/leroy_nav2/diagnostics/m22h/signed_rearm`.
Fake health node publishes invented boolean health on
`/leroy_nav2/diagnostics/m22h/synthetic_health`.
`m22h_signed_diagnostic_gate` combines the existing
`SessionVerifier` plus the M22A state machine through M22E's
**FAKE-only** `FakeSignedIngress`. It publishes **unconsumed**
`/leroy_nav2/diagnostics/m22h/accepted_twist` and
`/leroy_nav2/diagnostics/m22h/gate_status`.

The M22E test sink is `FakeSerialTransport`, a Python list.
Even after a legitimate signed rearm and 1-second healthy dwell,
nonzero wheel packets reach **only that fake test list**.
An independent ACTUAL `LeroyBridgeNode` is constructed with
`serial.Serial` replaced BEFORE constructor call with M22F's
strict `MemoryCDC` fake accepting only `m22f://memory`.
Production `_send_command()` still enforces M22D zero-wheel
allowlist and M22G terminal serial fault latch. Raw `/cmd_vel`
is still unsubscribed. No signed ROS subscriber or motion enable
is added to the production bridge node in this milestone.

## Acceptance cases

1. Startup remains latched and publishes exact-zero diagnostic Twist
2. Fresh signed fake operator grant and 1s continuous dwell permits
   bounded nonzero Twist on diagnostic topic ONLY
3. Actual production bridge `_send_command(nonzero)` still rejects
   every wheel packet; fake CDC sees no nonzero write
4. Forged/wrong-key signed ROS command latches and publishes zero
5. Healthy valid ROS commands cannot automatically rearm
6. A newly constructed verifier rejects previous-session signed data
7. Fake CDC write error terminally latches real bridge and suppresses
   further keepalives/STOP writes
8. Audit never includes session IDs, MACs or keys, and asserts that
   output is diagnostic-only and no physical device access occurred

## Trust boundary — IMPORTANT

**M22H does not complete production signed command integration.**
The signer and verifier are test-only and share memory for the
synthetic test secrets. DDS node names are NOT source authentication,
there is no operator identity/credential provisioning or secured
rearm endpoint, and signed ROS envelope origin is not protected by
a deployable interprocess trust boundary. Health is invented, not
live DepthART/XRSLAM. ROS topics are diagnostic-only.
The production bridge `node.py` and M22D `command_ingress.py`
are not modified. There is NO approved nonzero motion path.

Physical STM32 CDC opening itself may energize wheels/manipulator
independent of the software command guard. Do not start the real
`leroy-ros` service, map any `/dev` device, invoke `robot.launch.py`,
or consider this as evidence of a physical STOP acknowledgement.
Hardware protective design remains separately necessary but outside
the current software development scope.

## Build on Arch rootless Podman

Use the existing operator-validated image
`localhost/leroy-sensors:m22g-terminal-fault-selftest`.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22h-signed-ros-fake-bridge-arbitration
  git pull --ff-only origin feature/m22h-signed-ros-fake-bridge-arbitration
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m22g-terminal-fault-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m22h-selftest \
    -t leroy-sensors:m22h-signed-ros-selftest .
) 2>&1 | tee m22h_no_device_build.log
M22H_BUILD_EXIT=${PIPESTATUS[0]}
echo "M22H_BUILD_EXIT_CODE=$M22H_BUILD_EXIT"
```

Require `M22H_SIGNED_ROS_IMPORT_SELFTEST=PASS`, build exit 0.

## Run, strictly no devices

Only if the build passed:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m22h-signed-ros-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m22h-signed-ros-selftest
) 2>&1 | tee m22h_no_device_simulation.log
M22H_SIM_EXIT=${PIPESTATUS[0]}
echo "M22H_SIM_EXIT_CODE=$M22H_SIM_EXIT"
```

Required acceptance markers:

```text
M22H_SIGNED_ROS_STARTUP_INHIBIT=PASS
M22H_SIGNED_REARM_DIAGNOSTIC_FORWARD=PASS
M22H_PRODUCTION_CDC_LOCKOUT=PASS
M22H_FORGED_COMMAND_INHIBIT=PASS
M22H_NO_AUTOMATIC_REARM=PASS
M22H_SESSION_RESTART_REPLAY_REJECT=PASS
M22H_BRIDGE_TERMINAL_CDC_INHIBIT=PASS
M22H_ROS_SIGNED_BRIDGE_SELFTEST=PASS
M22H_VERDICT=SIMULATED_SIGNED_ROS_DIAGNOSTIC_ONLY
M22H_DEVICE_ACCESS=NONE
M22H_SIM_EXIT_CODE=0
```

## Subsequent software milestones

After this signed diagnostic rehearsal, the remaining work is
credential and operator-grant provisioning, independently trusted
timestamp/owner integrity, integration into a single enforcing
production bridge command ingress, fault/restart arbitration,
synthetic full-process crash tests, and eventual live sensor inputs.
Do not replace the M22D zero-only guard until these are verified
under a new explicit software release and separately approved
physical test plan.

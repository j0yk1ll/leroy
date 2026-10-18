# M22J — Bridge-owned signed audit seam, motion still inhibited

## Baseline

M22I operator Arch offline regression **49/49 PASS**:
16 isolated synthetic signer process tests, 23 M22E signed-session
tests and 10 M22G production source fake serial fault tests.
`M22I_DEVICE_ACCESS=NONE`, `M22I_OFFLINE_EXIT_CODE=0`.
PR #89 merged to `main` at
`b4b753f4aae0d8052d3c131a32898bb27afa5ca4`.

## Purpose and scope

Provide a minimal *bridge-owned, non-actuating* signed candidate
inspection method in real `LeroyBridgeNode`. M22H only exercised
the signed ROS gate BESIDE the bridge; M22I split synthetic signer
roles into separate OS processes. M22J introduces an explicit
auditing seam in the real bridge SOURCE while preserving the
unconditional production M22D nonzero-motion lockout.

## Changed source

- `src/leroy_bridge/leroy_bridge/bridge_signed_audit.py` (new)
  is a ROS-free, serial-free `BridgeSignedAudit` wrapper around
  the exact M22E `SessionVerifier`. It verifies HMAC/session/owner/
  timestamp/sequence for a motion envelope and independently
  verifies an operator-signed rearm envelope. The immutable result
  has only diagnostic booleans and reason strings, with
  `physical_motion_authorized=False` and `diagnostic_only=True`.
  **It never constructs motor PWM or sends a serial command.**
- `src/leroy_bridge/leroy_bridge/node.py` adds exactly one method,
  `_inspect_signed_diagnostic(audit, motion=..., rearm=..., now_ns=...)`,
  which accepts an exact `BridgeSignedAudit` instance and returns
  the audit result. It never calls `_send_command` or a serial writer.
  No ROS subscriptions are created, no keys provisioned, and no
  authorizing ingress is instantiated by the normal bridge startup.
- `src/leroy_bridge/test/test_m22j_bridge_signed_audit.py` adds
  14 pure-Python tests, including AST compilation of that EXACT
  method from production `node.py` with a poisoned serial writer.
- `m22j_real_bridge_signed_audit_ros_selftest.py` instantiates
  real `LeroyBridgeNode` against strict M22F injected fake CDC
  and tests signed envelopes from M22I's separate fake signer
  processes, replay, restart/session rotation, zero CDC writes
  during signed audit, retained M22D nonzero command rejection,
  and healthy shutdown on fake CDC only.
- `Dockerfile.m22j-selftest` and `m22j_no_device_run.sh`
  package the latest bridge source and strict fake-only lifecycle.

## What this does NOT solve

**Signed envelope verification is not operational permission.**
The bridge audit result deliberately never authorizes physical
motion, even if BOTH signatures are valid. No manual rearm dwell
or real sensor health assertion is represented by that audit result.
The preexisting M22A/M22E gate models those behaviors in fake tests.

The parent M22I test supervisor STILL knows both ephemeral keys,
runs under the same OS account as the signer children, and can
request synthetic operator grants. No authenticated human operator,
secure key custody, true process integrity, trusted ROS publisher
identity, real sensor freshness or physical STOP acknowledgement
is established. The package has NO external signed ROS subscriber.
Production `leroy_bridge/node.py` still opens real MCU serial if
launched normally: **do not run the robot or real bridge**.
Opening real STM32 CDC can energize wheels/manipulator without
a wheel command. The production CDC writer remains zero-only, but
that is not hardware protection.

## Offline run — Arch, ROS-free, no devices

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22j-bridge-signed-audit-seam
  git pull --ff-only origin feature/m22j-bridge-signed-audit-seam
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  export PYTHONPATH="$PWD:$PWD/src/leroy_bridge:$PWD/src/leroy_bridge/test"
  python3 -u src/leroy_bridge/test/test_m22j_bridge_signed_audit.py
  python3 -u m22i_isolated_role_signing_test.py
  python3 -u src/leroy_bridge/test/test_m22g_transport_fault.py
  python3 -u src/leroy_bridge/test/test_m22d_command_ingress.py
  python3 -m pytest -q \
    src/leroy_bridge/test/test_xrslam_contract.py \
    src/leroy_bridge/test/test_motor_pwm.py \
    src/leroy_bridge/test/test_clock_sync.py
) 2>&1 | tee m22j_bridge_audit_offline.log
M22J_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22J_OFFLINE_EXIT_CODE=$M22J_OFFLINE_EXIT"
```

Require M22J 14/14, M22I 16/16, M22G 10/10, M22D 14/14,
bridge pytest 20/20; **74/74 PASS**,
`M22J_BRIDGE_SIGNED_AUDIT_SELFTEST=PASS`,
`M22J_PRODUCTION_MOTION_ENABLED=FALSE`,
`M22J_DEVICE_ACCESS=NONE`, `M22J_OFFLINE_EXIT_CODE=0`.

## No-device ROS integration build

Requires previously validated local
`localhost/leroy-sensors:m22h-signed-ros-selftest` base image.

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m22h-signed-ros-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m22j-selftest \
    -t leroy-sensors:m22j-bridge-signed-audit .
) 2>&1 | tee m22j_no_device_build.log
M22J_BUILD_EXIT=${PIPESTATUS[0]}
echo "M22J_BUILD_EXIT_CODE=$M22J_BUILD_EXIT"
```

Require `M22J_REAL_BRIDGE_AUDIT_IMPORT_SELFTEST=PASS`, build exit 0.

## No-device ROS integration run

Only after successful build, NEVER mount serial or camera devices:

```bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m22j-audit-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m22j-bridge-signed-audit
) 2>&1 | tee m22j_no_device_simulation.log
M22J_SIM_EXIT=${PIPESTATUS[0]}
echo "M22J_SIM_EXIT_CODE=$M22J_SIM_EXIT"
```

Required acceptance markers:

```text
M22J_REAL_BRIDGE_FAKE_CDC_STARTUP=PASS
M22J_SIGNED_ROLE_AUDIT_NO_CDC=PASS
M22J_REPLAY_AUDIT_FAIL_CLOSED=PASS
M22J_ROTATED_SESSION_REJECT=PASS
M22J_NEW_SESSION_STILL_INHIBITED=PASS
M22J_M22D_NONZERO_CDC_LOCKOUT=PASS
M22J_REAL_BRIDGE_FAKE_SHUTDOWN=PASS
M22J_REAL_BRIDGE_SIGNED_AUDIT_SELFTEST=PASS
M22J_VERDICT=INJECTED_SIGNED_AUDIT_FAKE_CDC_ONLY
M22J_DEVICE_ACCESS=NONE
M22J_SIM_EXIT_CODE=0
```

## Future work

The next software milestone must implement secure operator grant
provisioning, explicit supervised command authorization, trusted
sensor freshness, mutually exclusive source identity, and ROS
transport isolation before a true signed ingress can be enabled.
Even then production CDC nonzero commands must stay disabled
until the software/hardware safety case is independently reviewed.

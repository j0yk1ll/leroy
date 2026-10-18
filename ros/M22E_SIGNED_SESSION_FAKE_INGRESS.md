# M22E — Signed session and explicit rearm, simulated serial ONLY

## Starting point and objective

M22D passed operator Arch host validation: **49/49** regression tests
(14 M22D, 15 M22C, 20 bridge pytest). PR #83 merged into main at
`26284ad97105e6514ef6ae3fbbc0211008b7717a`.

The long-term goal is to let the bridge accept ONLY fresh, exclusively
authorized motion. This first M22E increment defines and adversarially
tests the **message/session authentication contract** using signed
synthetic envelopes and the existing mecanum PWM conversion algorithm.
The physical actuator ingress remains deliberately locked by M22D.

## What's implemented

- `src/leroy_bridge/leroy_bridge/authorized_session.py` defines a
  versioned canonical HMAC-SHA256 command envelope and independently
  authenticated manual-rearm envelope. Both are bound to a random
  128-bit session ID generated when a verifier is created.
- Motion and operator grants use **distinct 32-byte keys**. A motion
  envelope binds owner, session ID, uint64 sequence, host-monotonic
  generation time, deadline, vx/vy/wz, protocol domain and MAC.
- Rearm binds session ID, monotonically increasing counter, generation
  time, deadline, distinct domain and operator-key MAC.
- Freshness allows only generation <= now <= expiry with
  expiry-generation <=250ms. Duplicated/nonmonotonic sequences and
  generation times are rejected; rearm counters and times also must
  increase. All timestamp comparisons assume one trusted monotonic
  clock. This assumption holds inside the synthetic test process,
  but is NOT yet established for distributed ROS publishers.
- `m22e_fake_signed_ingress.py` combines the verifier with the existing
  M22A `DiagnosticAuthority` (fault latch and 1s rearm dwell), existing
  production `leroy_bridge.motor_pwm.mecanum_twist_to_pwm` math, and
  M22C's in-memory-only fake serial sink. Every denied command sends
  an exact zero PWM to the FAKE receiver. Fake write errors terminate
  the fake session without automatic retry or rearm.
- `m22e_signed_session_test.py` contains **23 offline tests** checking
  initial inhibit, accepted fake command after an explicit signed rearm,
  new-session invalidation, tampered fields, forged keys and grants,
  old/reordered/expired/future commands, wrong owner or source graph,
  lost sensor health, no automatic rearm, grant replay, evaluator stalls,
  nonfinite signing, fake CDC errors and unchanged production lockdown.

## Important security and deployment caveats

**HMAC alone is not a deployable operator-authorization system.**
This code supplies test-only signing helpers, not protected key storage,
credential provisioning, trusted ROS source binding, operator UI,
multi-process monotonic-clock validation, process-lifetime supervision,
or secure rearm. Keys in tests are intentionally fixed FAKE values.
ROS DDS node metadata is not authentication. Treat all sensor health
values in the tests as invented booleans, not live XRSLAM/DepthART.

**M22E does not modify `leroy_bridge/node.py` or the M22D zero-wheel
serial allowlist.** There is NO new ROS subscriber or live nonzero
`set_wheels` path; real bridge `_send_command` must continue to reject
any nonzero command, even when produced by a valid test-only envelope.
The sink used here is a Python list of fake serial bytes; no STM32 port
or physical robot can receive them. The synthetic 0.15m/s, 0.30rad/s,
250ms and 1s bounds are provisional, not calibrated robot limits.
Fake serial output and test rearming are NOT an assertion of physical
STOP acknowledgment, actual torque removal, or safe autonomous motion.

## Arch offline validation — no container or device

Preserve unrelated local changes/deletions. Do not start `leroy-ros`,
map devices, invoke `leroy_bridge`, or open the STM32 CDC port.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22e-signed-session-fake-ingress
  git pull --ff-only origin feature/m22e-signed-session-fake-ingress
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  export PYTHONPATH="$PWD:$PWD/src/leroy_bridge"
  python3 -u m22e_signed_session_test.py
  python3 -u src/leroy_bridge/test/test_m22d_command_ingress.py
  python3 -u m22c_fake_serial_boundary_test.py
) 2>&1 | tee m22e_signed_session_offline.log
M22E_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22E_OFFLINE_EXIT_CODE=$M22E_OFFLINE_EXIT"
```

Acceptance: M22E `Ran 23 tests`, `OK`,
`M22E_SIGNED_SESSION_FAKE_INGRESS_SELFTEST=PASS`,
`M22E_VERDICT=OFFLINE_SIGNED_SESSION_FAKE_CDC_ONLY`,
`M22E_PRODUCTION_NONZERO_CDC_ENABLED=FALSE`,
`M22E_DEVICE_ACCESS=NONE`, M22D 14/14 PASS, M22C 15/15 PASS,
and `M22E_OFFLINE_EXIT_CODE=0`.

## Next work after host validation

M22F should integrate the verified command protocol and the real
`LeroyBridgeNode` lifecycle using **injected fake transport and
isolated ROS process tests**. It must prove a single ingress route,
single serial writer, watchdog/fault latch, crash/restart invalidation,
and authenticated operator/session provisioning without ever mapping
the physical CDC port. That engineering milestone is needed BEFORE
replacing M22D's zero-wheel-only production write guard.
Live sensor integration follows the software safety phase. The
hardware protective architecture is acknowledged as necessary but
remains outside the current software engineering scope.

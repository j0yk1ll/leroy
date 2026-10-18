# M22I — Interprocess synthetic signer roles and fresh-session rotation

## Previous milestone

M22H isolated ROS 2 diagnostic signing/rearming **PASS** on the
operator's Arch host. All requested acceptance markers passed; report:
24 ROS diagnostic status receipts, 24 ROS diagnostic Twist receipts,
24 fake-only PWM writes, **zero actual bridge nonzero CDC writes**,
terminal CDC fault latched, no physical device access, exit code 0.
PR #88 merged into `main` at
`90469c4822a983da80c14d9579ad7e65019294a6`.

## Why this milestone exists

M22H placed the fake controller, fake operator signer, and verifying
gate inside one Python process with TEST-CONSTANT HMAC keys.
A deployable command-authority architecture needs separately governed
identity, key custody, owner isolation and operator approval.
M22I begins testing **OS process separation**, still entirely offline
and without implying that actual identity or credential security
has been implemented.

## Implementation

- `m22i_isolated_role_signing.py`: a pure-Python `spawn` multiprocessing
  fixture. A parent test supervisor generates TWO independent ephemeral
  32-byte secrets with `secrets.token_bytes(32)`, creates a fresh M22E
  `SessionVerifier`, and spawns a motion signer and a rearm signer as
  different OS processes.
- Each child receives exactly its own ROLE key and the current session
  ID over Python multiprocessing startup IPC, and responds to limited
  `(role, uint64 counter, synthetic monotonic timestamp)` requests.
  A motion signer cannot issue rearm grants; an operator signer cannot
  issue motion envelopes. Messages returned to the parent are signed
  envelope dataclasses, not raw key bytes.
- Explicit `rotate_session()` closes both children, replaces both
  ephemeral keys and creates a new verifier ID. Old-session motion
  envelopes and rearm grants are rejected; no implicit motion rearm.
- An unavailable or crashed child raises `SignerUnavailable` instead
  of returning a cached signed command. The M22E fake ingress then
  inhibits the missing proposal, using M22A's latch and zero-output
  contract. No test signer communicates through ROS or USB.
- `m22i_isolated_role_signing_test.py`: 16 offline `unittest` cases
  check distinct process IDs, role-specific signature verification,
  denial of cross-role and malformed requests, no raw keys in normal
  return envelopes, replay and freshness, tampering, restart rotation,
  process close/crash, explicit rearm/dwell to fake PWM only, missing
  publisher latching, and production zero-wheel lockout.

## Security limitations — read before any future integration

**M22I is NOT production key provisioning or operator authentication.**
The test supervisor owns both secrets, can request arbitrary operator
grants, and injects a synthetic timestamp. The child processes run
with the same operating-system user and protection domain. IPC is a
local Python Pipe, NOT an authenticated or confidential security
boundary between mutually hostile processes. There is no protected
credential vault, secret rotation policy, peer credential check,
approval UI, TPM, managed secret store, ROS-Security enclaves or
verified hardware clock. The secrets are random rather than fixed
fixtures, but this only models a future lifecycle. OS process
separation alone does not authenticate an independent human operator.

No signed message is consumed by the actual bridge. All nonzero
commands remain in M22E's in-memory `FakeSerialTransport` test sink.
Production `leroy_bridge/node.py` and M22D `command_ingress.py` are
UNCHANGED. The real bridge continues to reject every nonzero
`set_wheels` packet. No ROS, camera, STM32 CDC, motors, or physical
device mapping are required. Opening a real STM32 CDC port may
energize the robot independent of software and is prohibited here.

## Run on Arch without hardware, ROS or Podman

The Python interpreter needs to support `multiprocessing` spawn.
Do not start production `leroy-ros` or access `/dev/ttyACM*`.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22i-isolated-role-signing
  git pull --ff-only origin feature/m22i-isolated-role-signing
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  export PYTHONPATH="$PWD:$PWD/src/leroy_bridge"
  python3 -u m22i_isolated_role_signing_test.py
  python3 -u m22e_signed_session_test.py
  python3 -u src/leroy_bridge/test/test_m22g_transport_fault.py
) 2>&1 | tee m22i_no_device_offline.log
M22I_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22I_OFFLINE_EXIT_CODE=$M22I_OFFLINE_EXIT"
```

Acceptance: `Ran 16 tests`, `OK`,
`M22I_INTERPROCESS_SIGNER_SELFTEST=PASS`,
`M22I_VERDICT=SYNTHETIC_EPHEMERAL_ROLE_SEPARATION_ONLY`,
`M22I_OPERATOR_AUTHENTICATION_IMPLEMENTED=FALSE`,
`M22I_PRODUCTION_MOTION_ENABLED=FALSE`,
`M22I_DEVICE_ACCESS=NONE`, 23/23 M22E regression PASS,
10/10 M22G regression PASS, `M22I_OFFLINE_EXIT_CODE=0`.

## Follow-up

M22J must translate the proven fake-only message/session behavior into
an **explicitly isolated trusted command-ingress interface** for
the production bridge, initially verified with injected FAKE CDC.
Real key custody, operator identity/consent, independent ROS
publisher/process authenticity, sensor integrity and restart
authorization still require their own threat model and tests.
Never remove M22D's zero-wheel guard merely because these synthetic
role tests pass. Physical protective design remains a separate
precondition for all future real-motion experiments.

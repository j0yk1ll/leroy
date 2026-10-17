# M22A — Disconnected software command-authority state machine

## Baseline verified

Consolidation PR #79 was verified merged into main at
`a0d35d6a600d065e408473ee7bfb044c8f348099`.
Software-safety engineering is the project's immediate focus, followed
by live sensor integration and later autonomous operation.

## Why this comes before the bridge

The hardware-capable `leroy_bridge` subscribes directly to `/cmd_vel`.
The M14 and M15 software gate outputs are strictly diagnostic and
unconsumed; their replay/source checks are not bridge command ownership.
M22A defines a standalone, actively selecting command decision core,
but does NOT connect it to any physical command topic, serial device,
motors, Nav2 controller or bridge. This enables adversarial acceptance
testing before implementing real command authority.

## Explicit pure-Python contract

`m22a_diagnostic_authority.py` exports `DiagnosticAuthority`,
`Inputs`, `Proposal`, `Decision` and ZERO. It consumes one new
synthetic proposal per evaluation, with a single expected owner label,
an incrementing uint64 sequence and increasing host-monotonic generation
timestamp. A missing, duplicated, replayed, future-stamped, stale,
nonfinite, nonplanar, or illustrative-limit-violating proposal,
bad graph ownership, unhealthy input, or time regression LATCHES inhibit.
Every blocked decision outputs the exact-zero tuple.

Startup is latched. Once healthy, an explicit caller-triggered
`operator_rearm=True` starts a 1-second synthetic healthy dwell with at most 200ms between evaluations.
Rearming does not forward a proposal immediately, and any fault
requires another deliberate rearm. Repeat rearm while armed is
rejected. Zero output is a software decision only, NOT physical stop.

The 0.15m/s x/y, 0.30rad/s yaw and 250ms proposal-age bounds are
borrowed as SYNTHETIC policy fixtures, NOT calibrated Leroy motion
limits. ROS graph owner labels are NOT authenticated identities.
The sender's `generated_mono_ns` can only be meaningful if the sender
and observer share an authenticated same-host monotonic timebase.
Real operator authentication, session binding, publisher identity,
clock integrity, lifetime and fault-reset handling require separate
ROS integration work. This version does NOT enforce motor actuation.

## Offline test

Fourteen pure-Python unittest cases exercise initial inhibit,
explicit rearm/dwell, sequence and generation-time replay,
clock reversal, evaluator stalls while rearming or armed, missing/duplicate/wrong owner, wrong proposal owner,
every health input, stale/future proposals, NaN/infinity/invalid axes,
missing proposal, no automatic rearm after a fault, repeated rearm,
invalid sequence and rearm types.

From the Leroy checkout on Arch:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22a-software-command-authority-core
  git pull --ff-only origin feature/m22a-software-command-authority-core
  cd ros
  PYTHONDONTWRITEBYTECODE=1 python3 -u m22a_diagnostic_authority_test.py
) 2>&1 | tee m22a_authority_offline.log
M22A_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22A_OFFLINE_EXIT_CODE=$M22A_OFFLINE_EXIT"
```

Expected `Ran 16 tests`, `OK`,
`M22A_SOFTWARE_AUTHORITY_SELFTEST=PASS`,
`M22A_VERDICT=SYNTHETIC_DIAGNOSTIC_AUTHORITY_ONLY`,
`M22A_DEVICE_ACCESS=NONE`, `M22A_OFFLINE_EXIT_CODE=0`.

## Known limits and next engineering gates

1. Connect this candidate to a diagnostic-only isolated ROS graph
   with known command producer identities and adversarial replay.
2. Design the actual bridge-side **single owner** binding,
   command freshness protocol and non-bypassable software gate;
   validate with a FAKE serial bridge, no physical devices.
3. Integrate and validate real sensor data separately. Only after
   the software-side mechanisms have been reviewed and tested
   should operational motion tests be considered.

This is a software-safety-only milestone. Hardware protective
mechanisms are acknowledged as important but are outside the
current engineering scope. No live hardware use, motor actuation,
or real camera–IMU calibration is authorized by M22A.

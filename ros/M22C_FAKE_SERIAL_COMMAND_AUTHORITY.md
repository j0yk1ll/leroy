# M22C — Software command authority driving a FAKE serial wheel sink

## Prerequisite

M22B passed the user's Arch isolated ROS command authority simulation:
16/16 M22A tests PASS, six M22B adversarial ROS scenarios PASS,
68 gate evaluations, 68 Twist + 68 status receipts, final LATCHED,
zero /cmd_vel publishers, no physical devices, exit code 0.
PR #81 verified merged into main at
`d0b38bae54a053e255840b10d39260d67a84d801`.

## Why this test is necessary

The real `leroy_bridge` node still subscribes to `/cmd_vel` directly
and turns Twist into four PWM duties then serial `set_wheels ...`.
M22A's pure software authority and M22B's ROS diagnostic gate are
NOT inserted in the deployed motor path. Before modifying production
bridge command ownership, prove its MOTOR PWM conversion method and
serial packet framing are usable behind a fake authorization boundary.

M22C introduces a standalone `SyntheticBridgeBoundary` and
`FakeSerialTransport` written in pure Python, with no ROS, pyserial
or hardware access. The synthetic boundary OWNS a DiagnosticAuthority
instance and calls the existing Leroy bridge
`leroy_bridge.motor_pwm.mecanum_twist_to_pwm()` conversion function
on its accepted diagnostic velocities. The resulting
`set_wheels FL FR RL RR` bytes go to a memory list ONLY.
On every blocked decision it writes an exact-zero wheel packet
to the *FAKE* transport. On a fake write failure, it latches and
performs no further fake writes; no real stop acknowledgment is
claimed. PWM conversion failure also blocks and writes zero.

Fifteen regressions exercise: startup zero, uninterrupted explicit
rearm and existing mecanum PWM math, duplicate/missing/wrong source,
wrong claimed proposal owner, stale/future command generation,
duplicate sequence and generation timestamps, sensor loss requiring
fresh rearm, absent input after accepted output, NaN/overlimit
commands, evaluation stall, invalid wheel conversion configuration,
fake serial write error with terminal latch, fake CDC command
allowlist, and a static check that there is no hardware import
and the original bridge has NOT been modified.

## Scope limitations

**M22C is NOT a non-bypassable deployed gate.** Other code can call
the fake transport directly, and the production bridge still
accepts raw, unstamped Twist on `/cmd_vel`. The test deliberately
demonstrates candidate enforcement logic only. It does NOT prove
DDS publisher authentication, operator authentication, interprocess
freshness, MCU watchdog behavior, real serial error recovery,
physical stopping or safe motion.

The wheel geometry/PWM gain/cap mirrors the current *provisional*
bridge YAML as artificial math fixtures. 0.15m/s etc from M22A
are not validated physical robot limits. No code in this PR
opens the STM32 CDC port, starts leroy-ros, or changes motor firmware.
Hardware protective engineering remains out of scope for these
software tests and must be handled separately before real motion.

## Run on the Arch host — no ROS, no Podman, no devices

Run inside the repository checkout; preserve unrelated local edits
and the two previously reported local tracked-file deletions.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22c-fake-serial-authority-sink
  git pull --ff-only origin feature/m22c-fake-serial-authority-sink
  cd ros
  PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH="$PWD:$PWD/src/leroy_bridge" \
    python3 -u m22c_fake_serial_boundary_test.py
) 2>&1 | tee m22c_fake_serial_offline.log
M22C_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22C_OFFLINE_EXIT_CODE=$M22C_OFFLINE_EXIT"
```

Acceptance:

```text
Ran 15 tests
OK
M22C_FAKE_SERIAL_AUTHORITY_SELFTEST=PASS
M22C_VERDICT=SYNTHETIC_AUTHORITY_TO_FAKE_WHEEL_PROTOCOL_ONLY
M22C_DEVICE_ACCESS=NONE
M22C_OFFLINE_EXIT_CODE=0
```

## After M22C

Next software work: create and test a **genuinely non-bypassable**
bridge command ingress and owner/session/nonce freshness protocol,
including a fake-serial-injected real bridge node and restart /
transport-fault matrix. Do not install M22C as the live serial writer.
Only with separate real sensor integration and physical-test
authorization can later motor-capable operations be considered.

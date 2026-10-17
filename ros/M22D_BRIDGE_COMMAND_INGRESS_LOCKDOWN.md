# M22D — Lock down the actual bridge's legacy raw-velocity actuator path

## Verified baseline

Operator M22C: 15/15 synthetic fake CDC authority tests PASS,
`M22C_FAKE_SERIAL_AUTHORITY_SELFTEST=PASS`, no physical devices,
`M22C_OFFLINE_EXIT_CODE=0`. GitHub confirmed PR #82 merged into
`main` at `18202d888ceb82438f56011786da482dcfdb1a76`.

## Scope: production source changed; physical runner NOT executed

This is an ACTUATION-INHIBITING software change to the actual
`leroy_bridge` module. Its existing `/cmd_vel` ROS `Twist`
subscription and `_on_cmd_vel` handler are REMOVED. The old command
timeout timer and dead raw-velocity handler are also removed.

The single Python serial writer `_send_command()` now calls the new
ROS-free `leroy_bridge.command_ingress.command_to_bytes()` allowlist
BEFORE it reaches `.write()` or `.flush()`. The allowlist permits:

- exact `set_wheels 0 0 0 0` (zero-only wheel packet)
- `keepalive`
- `set_watchdog_ms` with canonical integer in 1..60000
- `set_imu_stream_hz` and `set_encoder_stream_hz` with canonical
  integer 0/10/20/50/100

ALL other CDC strings—including ANY nonzero wheel request,
unknown commands, newline injection, alternate whitespace, watchdog
disable, overlong numbers, and non-ASCII text—are rejected before
serial access. Invalid inputs cannot toggle a flag to enable nonzero
wheel output. No ROS parameter, environment flag, alternate command
subscription or new authorized motion input is introduced here.

The previous `wheel_velocity_to_pwm_gain: 12.75` and `max_pwm: 64`
remain in the repository YAML as *inert historical calibration
fixtures*. They cannot bypass the serial command allowlist. The
legacy `cmd_vel_timeout_s` parameter remains read-only for source
compatibility, but is NOT an active watchdog timer in M22D.
Bridge sensor telemetry, MCU clock diagnostics, odometry publishing
behavior and stop-on-shutdown code remain present.

## Regression coverage: 14 new offline tests plus prior suites

`ros/src/leroy_bridge/test/test_m22d_command_ingress.py` tests:

1. Canonical telemetry, keepalive and zero wheel command admission
2. Reject ANY nonzero wheel PWM
3. Reject command injection, noncanonical formatting and unknown opcodes
4. Reject disabling watchdog or invalid watchdog bounds
5. Reject invalid sensor rates, overlong numeric values and wrong types
6. Static AST check that production bridge does NOT subscribe to
   `/cmd_vel` and removed its raw callback/timeout handler
7. Static check that the sole production `.serial.write()` occurs
   in `_send_command`, with allowlist evaluation first
8–10. AST-compile the ACTUAL production `_send_command` method into
   an in-memory fake bridge, verifying rejection before write and
   fake serial write/flush failure handling
11. Verify healthy keepalive/watchdog/telemetry commands still reach fake
    CDC memory buffer
12. Verify direct fake bridge `_send_command(nonzero)` cannot bypass
    the allowlist
13. Verify allowlist module imports no ROS, pyserial or OS device
14. Validate positive watchdog and updated source guard references

Previously validated M22C's static bridge-source regression is
updated to require the new guarded production state. M22C's other
14 fake-authority tests remain unchanged. The existing
`test_xrslam_contract.py` source-level regression continues to
verify read-only parameters and notes that old PWM YAML values are
now inert. These additional suites should also pass.

## Run on Arch — do NOT start the bridge

Run in the existing checkout. No Podman, ROS, serial device, or
robot power is required. Preserve any unrelated local changes,
including previously reported deletions of
`ros/CAMERA_TIMESTAMP_FIX.md` and `utils/src/utils/__init__.py`.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22d-bridge-raw-cmd-ingress-lockdown
  git pull --ff-only origin feature/m22d-bridge-raw-cmd-ingress-lockdown
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  export PYTHONPATH="$PWD:$PWD/src/leroy_bridge"
  python3 -u src/leroy_bridge/test/test_m22d_command_ingress.py
  python3 -u m22c_fake_serial_boundary_test.py
  python3 -m pytest -q \
    src/leroy_bridge/test/test_xrslam_contract.py \
    src/leroy_bridge/test/test_motor_pwm.py \
    src/leroy_bridge/test/test_clock_sync.py
) 2>&1 | tee m22d_bridge_ingress_offline.log
M22D_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M22D_OFFLINE_EXIT_CODE=$M22D_OFFLINE_EXIT"
```

Require 14 M22D PASS; 15 M22C PASS; previous bridge test suites
pass; exact `M22D_BRIDGE_INGRESS_LOCKDOWN_SELFTEST=PASS`,
`M22D_VERDICT=RAW_CMD_VEL_REMOVED_ZERO_WHEEL_ALLOWLIST_ONLY`,
`M22D_DEVICE_ACCESS=NONE`, and `M22D_OFFLINE_EXIT_CODE=0`.

## What this does NOT prove

**No device has been opened or real bridge ROS process started.**
AST-executing its CDC method with in-memory fake serial is not a
whole-node lifecycle test. Other programs/firmware accessing the
MCU independently are outside the source-level ingress check.
Even a zero-only software serial writer does NOT make a physical
CDC port safe to open: USB/DTR effects can energize wheels or
manipulator. Unverified STOP ACK, process failure or serial loss
cannot establish an independent stop. Hardware protective
engineering is acknowledged but remains outside this software
milestone, and physical access still requires separate authorization.

M22D provides NO motion enablement path. This intentionally
breaks legacy ROS `/cmd_vel` motor control in any image subsequently
rebuilt from this source. **Do not rebuild/redeploy/launch the
production robot container as part of this test.** Software-only
development next needs an exclusive session-bound gate-to-bridge
command protocol, trusted timing/ownership semantics, deliberate
operator rearm, process restart and transport failure handling,
validated end-to-end against a fake MCU.

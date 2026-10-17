# M22B — Disconnected ROS 2 command-authority integration

## Baseline

M22A 16 pure-Python authority regressions passed on the Arch host:
`M22A_SOFTWARE_AUTHORITY_SELFTEST=PASS`,
`M22A_DEVICE_ACCESS=NONE`, `M22A_OFFLINE_EXIT_CODE=0`.
PR #80 is merged into main at da02d8a0496f407fc6474a1d18b9ecc4338e7ee3.
M21M derived Jazzy image was also validated on the operator's host.

## Actual new integration scope

M22B runs genuine ROS 2 publishers/subscribers in a single process
and isolated DDS domain 170. Its fake controller_server publishes
`std_msgs/String` JSON envelopes with synthetic owner, monotonic
generation nanoseconds, sequence and six command axes. The separate
`m22b_fake_health` publishes all-true SYNTHETIC health predicates.
The gate subscribes to these topics, uses M22A's pure decision core
and publishes the selected `geometry_msgs/Twist` to ONLY a new
unconsumed DIAGNOSTIC output. An auditor receives the real ROS Twist
and JSON status streams. The harness invokes manual rearm directly
in-process; there is NO remote rearm ROS topic, service or parameter.

Four topic contracts:

- /leroy_nav2/diagnostics/m22b/proposed_envelope
- /leroy_nav2/diagnostics/m22b/synthetic_health
- /leroy_nav2/diagnostics/m22b/gated_cmd_vel
- /leroy_nav2/diagnostics/m22b/authority_status

Five acceptance phases:

1. Startup remains latched and publishes exact-zero diagnostic Twist
2. Explicit TEST-ONLY rearm, continuous 10Hz evaluation and 1s
   healthy dwell allow bounded nonzero DIAGNOSTIC ROS Twist
3. Competing second publisher on fake proposal topic triggers
   BLOCKED_COMMAND_OWNERSHIP with exact-zero ROS output
4. Removing the competing publisher does NOT automatically rearm
5. After a fresh test-only rearm, stale generation stamp latches
   and subsequent missing proposals do not replay old velocity

The gate refuses ROS topic remapping, rejects unrecognized/duplicate
publisher graph identities, and never publishes or subscribes
to `/cmd_vel`. `leroy_bridge`, firmware, Nav2, XRSLAM, DepthART
and motor-capable robot bringup are NOT included in the test image.

**Important limitations:** Fake health booleans are not measured
sensor health. Fake controller ROS graph node name is not proof
of source identity, and sender-defined `generated_mono_ns` is not
an authenticated cross-process freshness guarantee. The in-process
test-only rearm is not a deployable operator control. Nonzero
diagnostic commands never reach the physical bridge. Software
control limits and timing remain provisional fixture choices.

## Build on Arch rootless Podman — no devices

Preserve any unrelated local modifications; a previous checkout
reported deletions in `ros/CAMERA_TIMESTAMP_FIX.md` and
`utils/src/utils/__init__.py`. Do not restore or reset those changes.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m22b-isolated-ros-command-authority
  git pull --ff-only origin feature/m22b-isolated-ros-command-authority
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m21m-producer-fault-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m22b-selftest \
    -t leroy-sensors:m22b-authority-selftest .
) 2>&1 | tee m22b_no_device_build.log
M22B_BUILD_EXIT=${PIPESTATUS[0]}
echo "M22B_BUILD_EXIT_CODE=$M22B_BUILD_EXIT"
```

Required build marker `M22B_ROS_IMPORT_SELFTEST=PASS`,
and build exit 0. This does not run real ROS callbacks during build.

## Isolated ROS simulation

Run only once build succeeds:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m22b-authority-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m22b-authority-selftest
) 2>&1 | tee m22b_no_device_simulation.log
M22B_SIM_EXIT=${PIPESTATUS[0]}
echo "M22B_SIM_EXIT_CODE=$M22B_SIM_EXIT"
```

Require M22A 16/16 PASS plus:
`M22B_STARTUP_INHIBIT=PASS`,
`M22B_REARM_AND_DIAGNOSTIC_FORWARD=PASS`,
`M22B_DUPLICATE_PUBLISHER_INHIBIT=PASS`,
`M22B_NO_AUTOMATIC_REARM=PASS`,
`M22B_STALE_COMMAND_INHIBIT=PASS`,
`M22B_MISSING_MESSAGE_NO_REPLAY=PASS`,
`M22B_ROS_AUTHORITY_SELFTEST=PASS`,
`M22B_VERDICT=SIMULATED_ROS_DIAGNOSTIC_GATE_ONLY`,
`M22B_DEVICE_ACCESS=NONE`, `M22B_SIM_EXIT_CODE=0`.

## Next software-safety milestones

Before connecting to motors, real software authority needs an
enforcing source identity and epoch/sequence protocol, freshness
based on trusted generation timestamps or monotonic session time,
a single non-bypassable bridge command entrypoint, service/restart
fault management, and real fault/rearm arbitration. M22B is only
a no-device ROS communication rehearsal, NOT completion of those
requirements. Hardware safety mechanisms remain acknowledged as
necessary but outside this software engineering scope.

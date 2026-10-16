# M14 — Enforcing software command gate (isolated diagnostic deployment)

**Goal:** implement actual ROS 2 software proposal suppression,
not another shadow-only observer. M14 consumes the real Nav2 MPPI
controller's **hard-remapped, actual** `geometry_msgs/Twist`
proposals, independently observes fake odom/depth, dynamic odom-to-base TF,
and rolling costmap, and publishes the chosen output to a NEW, otherwise
unconsumed **diagnostic-only** gated topic. Every reject emits
**exact zero Twist**. The gate also publishes JSON state for
auditability. The ROS process will NEVER connect to Leroy's
hardware /cmd_vel in this milestone.

M14's actual software gate is a **functional component** intended
to be evolved toward Leroy's deployable command pipeline. The
next integration work must replace fake sensor sources, validate
all real data/protocol ownership, and explicitly review any
separate robot deployment. M14 tests are not physical safe-stop,
watchdog, emergency-stop, power isolation or collision guarantee.

## Existing scope and conservative configuration

Source: `/leroy_nav2/diagnostics/proposed_cmd_vel` (ACTUAL MPPI
Twist; only the offline `controller_server`). Source metadata
is checked by ROS graph inspection but is **not cryptographically
authenticated**. Extra/wrong/unknown publisher identity -> zero
until exactly one expected controller publisher is resolved.

Output: `/leroy_nav2/diagnostics/gated_cmd_vel`
(`geometry_msgs/msg/Twist`), an intentionally **unconsumed**
diagnostic topic. Neither ROS2 relative nor absolute `cmd_vel`
can map to it automatically: the Nav2 controller launch retains
both remaps `('cmd_vel', DIAGNOSTIC_CMD_TOPIC)`,
`('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)`. Gate startup verifies all
expected topic names to reject unauthorized remap attempts.

State: `/leroy_nav2/diagnostics/m14_gate_status`
(`std_msgs/msg/String` carrying per-cycle JSON; includes
reason, monotonic sample time, sequence, current nonzero input
state and actual *selected* output vector). It is a diagnostic
report, NOT an authorization topic or substitute for reading
the `Twist` output.

M7 fictional limits are retained: |vx|<=0.15 m/s,
|vy|<=0.15 m/s, |wz|<=0.3 rad/s; other axes must be
effectively zero. These values and the controller's synthetic
1m-square footprint are NOT measured physical Leroy limits.
A proposal is usable only while received within 0.4s; both
fake odometry, fake depth, and matching dynamic odom-to-base TF
need fresh receipt AND acquisition timestamps <=1.5s. Local costmap must be valid and recently
received. The software gate evaluates continuously at 10Hz
even if proposals stop arriving; it emits zero on every blocked
tick. If all inputs remain valid for the M7-derived **1s
rearm interval**, it resumes forwarding only a fresh bounded
proposal. A bad graph, source identity, invalid values, lost
sensor, timestamp regression, bad costmap, clock discontinuity,
or expired proposal resets rearming. The TF condition is a
freshly received, correctly framed, normalized synthetic transform;
it is not a full localization correctness assessment.

**Critical distinction:** the old M7 observer still emits
`WOULD_VETO_*` labels without changing MPPI output. M14
really publishes the selected **zero Twist** while vetoing;
the original MPPI proposal diagnostic stream remains unchanged.

## Contract and known limits

- The gate does not own the FollowPath action; a separate probe
  submits one short, synthetic action. Stopping/zeroing a diagnostic
  stream is NOT the same thing as cancelling a Nav2 goal.
- Publishing zeros is a **software command intention**, not proof
  of braking or successful delivery to an actuator. Neither a
  bridge nor any hardware ROS node is present.
- Without a message-generation timestamp, a fresh subscriber
  callback may still be old source data. ROS publisher names are
  not cryptographic identity. This is a current limitation and
  must be addressed in future real-robot protocols.
- No global planner, route coverage, localization recovery,
  teleop mux, collision monitor, measured footprint or physical
  stopping behavior is implemented by M14.
- Fault handling at this stage is automatic after a synthetic
  one-second recovery dwell; real-world manual-rearm and state
  ownership requirements from M13 remain unimplemented.
- A software zero message is *not* independent of a failed ROS
  runtime, transport, bridge or motor firmware. Physical safety
  is outside this software milestone; never represent M14 as
  replacing it.
- The repeated artificial inflation warning persists:
  `inflation_radius=0.65m < circumscribed=0.721249m`. No
  parameters are weakened to force a pass.

## Five new pieces + setup/build integration

1. `m14_gate_core.py`: ROS-free decision function, exact zero
   on rejection, all sensor/command freshness and one-second
   valid recovery dwell.
2. `m14_software_gate.py`: **actual ROS runtime node** with
   an input subscriber and diagnostic output publisher;
   implements 10Hz checks, graph constraints, JSON state.
3. `m14_gate_fault_fixture.launch.py`: six synthetic processes
   (M11 timed fake odom or depth, other fake source, obstacle
   adapter, real MPPI controller_server, gate node, inactive
   lifecycle manager) with hard diagnostic MPPI remaps.
4. `m14_gate_audit.py`: one real synthetic FollowPath action,
   triggered fake sensor blackout, read-only diagnostic output
   audit with 10Hz gate status and actual gated Twist receipts.
5. `test_m14_gate.py`: pure deterministic tests for health,
   nonfinite/nonplanar candidate zeroing, stale input, graph
   loss, rearm, runtime wiring and build constraints.
6. Console entry points and optional image install checks.
   No change to production Docker, firmware or real bridge.

## Build disconnected optional image

From the repository root:

```bash
git fetch origin
git switch feature/nav2-m14-enforcing-diagnostic-command-gate
git pull --ff-only origin feature/nav2-m14-enforcing-diagnostic-command-gate
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m14-gate .
```

The build must complete all optional Colcon/pytest suites and
verify the installed `m14_software_gate`, `m14_gate_audit`
console entries and the M14 launch file. No real sensors,
motors, serial bridge or production container are built or
modified. Preserve any unrelated local changes rather than
forcing branch checkout or cleaning files.

## Trial A: triggered fake depth blackout

**Terminal 1 (six synthetic processes):**

```bash
podman run --rm -it --name leroy-nav2-m14 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:m14-gate bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration m14_gate_fault_fixture.launch.py fault:=depth
  '
```

**Terminal 2 (configure, audit, finalize):**

```bash
podman exec leroy-nav2-m14 bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash

  state=""
  for i in $(seq 1 20); do
    state="$(ros2 lifecycle get /controller_server 2>/dev/null || true)"
    [ "$state" = "unconfigured [1]" ] && break
    sleep 1
  done
  test "$state" = "unconfigured [1]"
  if ros2 topic list | grep -Eq "^(/cmd_vel|/odom)$"; then
    echo "FAIL: motor-compatible topic exists" >&2
    exit 1
  fi
  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /controller_server activate

  ros2 run leroy_nav2_integration m14_gate_audit \
    --ros-args -p fault_mode:=depth

  ros2 lifecycle set /controller_server deactivate
  ros2 lifecycle set /controller_server cleanup
  ros2 lifecycle set /controller_server shutdown
  ros2 lifecycle get /controller_server
'
```

Expected: pre-fault M7 healthy and nonzero gated
`FORWARD_DIAGNOSTIC_ONLY`; fake depth `SOURCE PAUSED`,
sustained `WOULD_VETO_BLOCKED_DEPTH`, M14 gate
`BLOCKED_DEPTH`. **During sustained gate depth veto** with
MPPI's last proposal still nonzero, gate must have at least
three ticks with `input_nonzero=true` while
`output_nonzero=false` and exact zero output vector. The
separate observed gated `Twist` subscriber must receive
at least five zero samples (no nonzero) in the conservative
inner blocked interval. After depth returns and the recovery
dwell passes, nonzero diagnostic forwarding must resume.

Require `M14 GATE OBSERVATION: mode=depth ...`,
`blocked_gate_nonzero_ticks=0`, `PASS: M14 actual MPPI ...`,
`finalized [4]`. After controller finalization press
Ctrl+C in Terminal 1; **all six synthetic processes must
exit cleanly**. This is not a physical stop test.

If the probe fails, `set -e` skips shutdown. Preserve the
full Terminal 1 and 2 outputs. Stop the controller and
finish with Ctrl+C separately; never change the remaps,
bounds or input predicates just to force a pass.

## Trial B: triggered fake odometry blackout

Start a **fresh** disconnected container with the exact same
security options and `fault:=odom` in Terminal 1; run
Terminal 2 with `fault_mode:=odom`. Require M7
`WOULD_VETO_BLOCKED_ODOM`, gate `BLOCKED_ODOM`,
`blocked_gate_nonzero_ticks=0`, at least five
observed zero gated Twist during confirmed blocked
blackout, and at least one recovered diagnostic
nonzero forward, then `finalized [4]` and all six
clean fixture process exits.

## M14 acceptance boundary

M14 requires:
- build/pytest success in the optional image;
- Trial A and Trial B passing under `podman --network none`,
  no shared namespaces, capabilities, devices or host bridge;
- exactly one source controller diagnostic publisher and
  one gate publisher on each gated output/state topic;
- no motor-compatible `/cmd_vel` or normal `/odom`;
- stable zero output on veto despite continued nonzero
  actual MPPI diagnostic proposals;
- recovery only after healthy synthetic inputs and dwell;
- manual Nav2 controller lifecycle `finalized [4]` and
  all **six** processes exiting cleanly.

**Future integration is explicitly separate:** validate real
sensor acquisition, ROS graph authority and message-source
provenance, command-expiry semantics and dataflow before
reviewing a deployment route. M14 intentionally does not
implement a real motor interface or remove M12's unresolved
readiness gates. All safety-certificate claims remain out of
scope.

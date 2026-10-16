# M11 — Measure actual MPPI diagnostic output INSIDE fake-sensor blackout

M10's depth and odometry active-goal trials passed as read-only
experiments. However, the M10 counters (70/54 for depth; 80/59 for
odom) cover an observation period that extends beyond the five-second
fault. They are **not counts of nonzero velocities during the blackout**
and not evidence that a controller stopped safely.

M11 refines only measurement. **No robot, hardware, braking, gate,
watchdog, interlock, or live `leroy-ros` involvement.** MPPI still
hard-remaps both `cmd_vel` and `/cmd_vel` to the unconsumed synthetic
`/leroy_nav2/diagnostics/proposed_cmd_vel` topic. Source names
are ROS graph metadata, not cryptographic authentication.

## Design

Five synthetic fixture processes, identical controller configuration
and critic parameters to M10, with one selected fake sensor fixture.
M11 changes only that fake fixture's **Trigger response message**:
the M10 one-shot 5s pause is unchanged, but the service reply includes
`scheduled_start_mono` and `scheduled_end_mono` measured using the
same monotonic clock as the M11 observer process. These timestamps
describe the **armed pause schedule**, not when the fake publisher
timer first skips a sample. Depth fixture ticks about every 0.5s,
while fake odom ticks about every 0.1s.

M11's observer inherits all M9/M10 constraints, including:
- A short single synthetic FollowPath goal must be accepted and still
  active; a bounded nonzero **actual MPPI** diagnostic `Twist` and
  1s healthy M7 baseline are required BEFORE fault trigger.
- ROS graph needs one `controller_server` publisher of
  `geometry_msgs/msg/Twist`, namespace `/`; unknown DDS metadata
  never grants trust, duplicate/known wrong publishers fail.
- Synthetic depth/odom receipt and acquisition timestamps are
  checked by M7; stale source diagnosis takes priority over MPPI
  candidate freshness. Every received candidate is finite, planar
  and inside fictional M7 limits. No `/cmd_vel` or ordinary `/odom`
  topics, bridge, planner, real sensors or motors.
- One 5s fault is explicitly triggered after the pre-fault baseline.
  The expected `WOULD_VETO_BLOCKED_DEPTH` or
  `WOULD_VETO_BLOCKED_ODOM` must appear INSIDE the armed blackout
  window, sustained at least 0.5s. The fake service refuses rearming.
- Diagnostic collection continues through the scheduled return,
  whether the synthetic action aborts or remains active. Cancellation
  must be acknowledged or action confirmed terminal.

The new pure-Python metrics count, by **observer receipt time**:
- `scheduled_zero` and `scheduled_nonzero`: candidate samples in
  the armed [start,end) 5s window. A message may have been computed
  earlier; the Twist message has no timestamp.
- `inner_zero` and `inner_nonzero`: conservative interval
  [start+0.75s,end-0.25s), excluding uncertainty around fake fixture
  timer boundaries and ROS callback ordering.
- `after_veto_zero` and `after_veto_nonzero`: samples received
  after the **first expected M7 blocked label** while the scheduled
  blackout has not ended. The veto NEVER gates controller output.
- `veto_from_schedule_s`: delay between fake fault arming and
  detection of the first correct M7 veto.
- `veto_from_last_sensor_rx_s`: delay since latest pre-request fake
  sensor receipt timestamp.
- `veto_vs_recv_age_threshold_s`: difference versus the M6
  1.5s receipt-age threshold, for *approximate interpretation only*.
  The last pre-request stamp can precede the last sample delivered
  between the request and the first skipped fake timer callback,
  and acquisition ROS timestamps are checked separately. It is
  NOT an exact hazard reaction time or guaranteed bound.

Zero samples during blackout are legitimate evidence, not a failure.
Nonzero samples **do not** imply motor movement or real tracking.
A PASS confirms measurement + hypothetical stale-input veto ONLY.

## Build isolated optional image (host repo root)

```bash
git fetch origin
git switch feature/nav2-m11-blackout-window-metrics
git pull --ff-only origin feature/nav2-m11-blackout-window-metrics
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m11-window .
```

Build must pass the optional packages' Colcon/pytest and installed
`m11_timed_depth`, `m11_timed_odom`, `m11_blackout_probe`
executables and M11 launch-file existence checks. Production Docker,
`leroy-ros`, hardware and existing unrelated local dirty state
are not touched.

## Trial A — fake depth blackout

Terminal 1:

```bash
podman run --rm -it --name leroy-nav2-m11 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:m11-window bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration m11_timed_fault_fixture.launch.py fault:=depth
  '
```

Terminal 2:

```bash
podman exec leroy-nav2-m11 bash -lc '
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
    echo "FAIL: forbidden motor-compatible topic" >&2
    exit 1
  fi
  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /controller_server activate

  ros2 run leroy_nav2_integration m11_blackout_probe \
    --ros-args -p fault_mode:=depth

  ros2 lifecycle set /controller_server deactivate
  ros2 lifecycle set /controller_server cleanup
  ros2 lifecycle set /controller_server shutdown
  ros2 lifecycle get /controller_server
'
```

Require M7 healthy baseline, `M11 WINDOW:` start/end,
sustained `WOULD_VETO_BLOCKED_DEPTH` during the scheduled interval,
`M11 BLACKOUT METRICS:` with distinct zero/nonzero counts for all
three intervals and veto latency, `PASS: M11 synthetic depth`, and
controller `finalized [4]`.

Ctrl+C in Terminal 1 *after* manual lifecycle finalization; all
**five** synthetic fixture processes must exit cleanly (no SIGSEGV).
If the probe fails, `set -e` skips cleanup: attempt controller
deactivate/cleanup/shutdown manually if possible, Ctrl+C in fixture
terminal, preserve full logs. Never loosen safety/limits/remaps to
force a PASS.

## Trial B — fake odometry blackout

Start a **fresh** isolated container using Terminal 1 above with
`fault:=odom`. Run Terminal 2 with `fault_mode:=odom`. Require the
same acceptance sequence but `WOULD_VETO_BLOCKED_ODOM`, full sample
metrics, `PASS: M11 synthetic odom`, `finalized [4]`, and five
clean fixture exits. Never rerun in the previous finalized/triggered
container.

## Nonclaims and known limitations

This experiment is explicitly **not** a measurement of physical
stopping distance, actual safe braking latency, velocity suppression
by a safety interlock, cause of the Nav2 progress-checker abort, or
real-source authentication. Controller diagnostics are still free
to publish during stale sensor observations; nothing filters them.
Synthetic wheel motion is independent of MPPI control.

The known fictional inflation mismatch (0.65m vs 0.721249m
circumscribed footprint) remains unmodified; do not adjust it
to manufacture success. M11 acceptance requires the operator's
image test evidence and both independent ROS depth/odom trials.
No M12 or hardware/physical operation authorization.

# M10 — Triggered fake sensor outage during an accepted synthetic MPPI goal

This is a **diagnostic experiment**, not a safety interlock or proof
MPPI can brake on loss of depth/odometry. Nothing is connected to
motors, cameras, real wheel telemetry, `/cmd_vel`, production
`leroy-ros`, or an external robot network.

## Scope and distinction from M6–M9

- M6: periodic fake source blackout; no FollowPath goal.
- M7: pure Python hypothetical veto with freshness/rearm.
- M8: ROS shadow fault audit with *constant invented Twist*; no goal.
- M9: read actual MPPI diagnostic proposals during a short synthetic
  FollowPath action, but with *no intentional sensor dropout*.
- **M10:** after an *accepted fake FollowPath* and a source-verified,
  bounded, nonzero actual MPPI diagnostic proposal and 1s healthy M7
  baseline, explicitly trigger **one** 5s fake depth/odom blackout.
  Count any real MPPI diagnostics seen during and after M7's veto.
  Also observe whether Nav2 ends the action. NEVER assume the output
  goes quiet, never substitute a fabricated candidate, and never
  describe controller logs saying "stopping" as physical braking.

The trigger controls **only** one synthetic ROS fixture publisher:
`/leroy_nav2/diagnostics/m10_trigger_fake_fault`
(`std_srvs/srv/Trigger`). In depth mode the fake depth Image and
CameraInfo stop for 5s, retaining the old non-clearing obstacle map.
In odom mode the fake Odometry and dynamic odom→base_link TF stop
for 5s. The fake odom does not integrate the pause on resumption.
The service can be called only once per fixture. **The fixture itself
never starts a FollowPath action.**

The optional launch starts exactly five existing/synthetic processes
(only one selected fake fault node, the other normal fake source,
obstacle adapter, Nav2 controller_server with embedded odom rolling
costmap, and inactive lifecycle manager). It preserves the two M5/M6
controller remaps to **only**

```
/leroy_nav2/diagnostics/proposed_cmd_vel
```

and does not start M8's fabricated Twist, planner, bridge or hardware.

The M10 probe is a **subscriber, FollowPath action client and
fixture Trigger service client** only. It creates no ROS publisher.
It inherits M9's strict message bounds, M7 policy and graph identity:
one controller_server in / publishing geometry_msgs/msg/Twist; a
single unknown DDS metadata identity may wait up to 12s *before*
trigger, never considered valid. Identity loss after trigger fails.
It rejects unexpected /cmd_vel, normal /odom, bridge/navigation nodes,
or any nonfinite/out-of-bounds planar MPPI diagnostic Twist.

## Operator build — optional disconnected image

From repository root, preserve any unrelated local changes:

```bash
git fetch origin
git switch feature/nav2-m10-triggered-mppi-fault-observation
git pull --ff-only origin feature/nav2-m10-triggered-mppi-fault-observation
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m10-fault .
```

The build runs Colcon/pytest for the same five packages, checks M10
console executable registrations and requires the new M10 launch to
exist in the installed share directory. No production image changes.

## Trial A — fake depth failure, fresh disconnected container

Terminal 1:

```bash
podman run --rm -it --name leroy-nav2-m10 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:m10-fault bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration m10_active_fault_fixture.launch.py fault:=depth
  '
```

Terminal 2 (no manual fault trigger; the probe calls it exactly once
only after accepted goal and proven healthy M7 baseline):

```bash
podman exec leroy-nav2-m10 bash -lc '
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
    echo "FAIL: forbidden motor-compatible graph topic" >&2
    exit 1
  fi

  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /controller_server activate

  ros2 run leroy_nav2_integration m10_active_fault_probe \
    --ros-args -p fault_mode:=depth

  ros2 lifecycle set /controller_server deactivate
  ros2 lifecycle set /controller_server cleanup
  ros2 lifecycle set /controller_server shutdown
  ros2 lifecycle get /controller_server
'
```

Expected essential evidence (startup labels may differ):

```text
M10 MPPI SHADOW: WOULD_VETO_REARM
M10 MPPI SHADOW: HEALTHY_DIAGNOSTIC_ONLY
M10 FAULT: triggered depth fake blackout; goal was active before trigger
M10 MPPI SHADOW: WOULD_VETO_BLOCKED_DEPTH
M10 FAULT: sustained WOULD_VETO_BLOCKED_DEPTH
M10 OBSERVATION: post_trigger_mppi_samples=... post_veto_mppi_samples=...
                  rearm_seen=... recovered_with_proposals=... action=...
PASS: M10 synthetic depth blackout overlapped an initially active synthetic FollowPath goal; ...
finalized [4]
```

The probe does NOT demand a particular controller outcome under fault:
action may abort, complete, or remain active until the probe cancels
it. The explicit outcome and proposal counts are evidence, **not
a PASS for controller safe stopping**. It requires an initially
active goal, real MPPI nonzero pre-fault proposal, M7 health before
trigger and 0.5s persistent correct M7 veto, bounded observation,
then acknowledged cancellation or terminal goal. If anything fails,
capture exactly what happened—do not adjust fictional limits,
controller critics, missing-data acceptance, or remap to force PASS.

After terminal 2 succeeds, Ctrl+C in terminal 1. Require all **five**
fixture processes to finish cleanly, especially controller_server
and its embedded costmap (no SIGSEGV).

## Trial B — fake odometry failure

Start a **new** no-network no-device `leroy-nav2-m10` container using
the same Terminal 1 command with `fault:=odom`. In Terminal 2
repeat the manual lifecycle + audit with `fault_mode:=odom`.
Require `WOULD_VETO_BLOCKED_ODOM` sustained for 0.5s, distinct
action outcome + MPPI proposal counts, PASS, `finalized [4]`, and
five clean teardown exits.

If the probe fails after activation, `set -e` intentionally skips
lifecycle cleanup. Try manual lifecycle deactivate, cleanup and
shutdown if possible, then Ctrl+C in Terminal 1 and collect the logs.
Never repeat the one-shot trigger/goal in an already used fixture.
Never run against the live `leroy-ros` container or connect devices.

## Interpretation

**M10 PASS demonstrates that the independent M7 shadow audit
diagnosed stale synthetic inputs while actual MPPI diagnostics were
monitored for an accepted synthetic action.** It does not prove
MPPI stops output, cancels automatically, performs a safe-stop,
tracks the fake path, authenticates any DDS publisher, protects
a physical robot, or supplies an E-stop/command gate. Unstamped Twist
allows only receipt age, not source data age. A diagnostic veto is
merely a label: it does not filter MPPI's output.

The synthetic costmap may retain lethal obstacle marks during
blackout, but stale depth/odom is still stale. The prior arbitrary
inflation radius 0.65m < circumscribed radius 0.721249m remains
unfixed, with no physical calibration implied. No M11 or
physical motor testing without separate explicit authorization.

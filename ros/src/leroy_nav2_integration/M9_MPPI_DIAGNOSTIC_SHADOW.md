# M9 — Offline actual MPPI diagnostic-proposal shadow audit

M8 validated M7 policy against a **fabricated constant Twist** on
`/leroy_nav2/diagnostics/synthetic_shadow_candidate`. M9 is a
separate, narrower **nominal proposal-source** test. It observes the
**actual Nav2 MPPI controller_server's proposed Twist**, generated
only during one short synthetic `FollowPath` action. Everything is
still disconnected, offline and simulated.

## Exactly what M9 does

Use the unmodified M5 `m5_controller_fixture.launch.py` (five
synthetic processes, fake odometry, fake depth/obstacle adapter,
Nav2 controller and internal non-clearing rolling costmap, manual
controller lifecycle). That fixture hard-remaps **both** relative
`cmd_vel` and absolute `/cmd_vel` to:

```text
/leroy_nav2/diagnostics/proposed_cmd_vel
```

M9's new executable `mppi_shadow_probe` creates **zero ROS
publishers**. It is a subscriber and synthetic-only action CLIENT.

It observes the remapped controller `geometry_msgs/Twist`, fake
odometry, fake obstacle points and the embedded costmap. It requires
the actual diagnostic publisher endpoint to be exactly one
`controller_server` in namespace `/`, message type
`geometry_msgs/msg/Twist`; a missing publisher or one with DDS
`_NODE_NAME_UNKNOWN_` metadata gets up to 12 seconds *after*
the fake action begins for discovery. Unknown identity is never
evidence of valid MPPI output; duplicate or known wrong endpoints
fail immediately. **ROS node-name metadata are not cryptographic
authentication**.

The probe waits for synthetic odometry, fake obstacle points and a
marked+unknown rolling costmap, then sends an M5-style seven-pose
short lateral `FollowPath` goal around the current fake odom pose.
No planner or BT navigator is launched and the goal is never sent to
a physical robot. It requires:

1. Controller accepts the single synthetic goal.
2. At least one post-goal, nonzero, finite and bounded planar
   **actual MPPI** diagnostic proposal appears.
3. The unchanged M7 `ShadowWindow` checks real ROS acquisition and
   monotonic receipt timestamps for fake odom/depth, valid costmap,
   candidate freshness and fictional bounds, and returns
   `HEALTHY_DIAGNOSTIC_ONLY` after its uninterrupted 1 second rearm.
4. The goal's cancellation is acknowledged or confirmed terminal.
5. No `/cmd_vel` or ordinary `/odom` topic appears, and clean
   Nav2 lifecycle shutdown follows.

The action observation deadline after goal acceptance is 16s.
A missing/unknown publisher cannot produce `HEALTHY_DIAGNOSTIC_ONLY`.
The output prints `M9 MPPI SHADOW: ...` label transitions and
`PASS: M9 actual MPPI remapped diagnostic Twist ...`.

**M9 does NOT inject sensor blackouts while MPPI has a goal.**
The M6/M8 no-goal sensor outages and M9 nominal real-proposal path
must not be interpreted as proof of what MPPI does on sensor loss.
A proposal that passes the fictional M7 bounds is never permission
to drive; MPPI output does not go to motors or any bridge.
Unknown space, footprint, fake calibration and inflation-radius
advisory 0.65m vs 0.721249m remain unchanged.

## Build and verify (host at repo root)

```bash
git fetch origin
git switch feature/nav2-m9-real-mppi-shadow-observer
git pull --ff-only origin feature/nav2-m9-real-mppi-shadow-observer
cd ros
podman build --no-cache -f Dockerfile.nav2 -t leroy-nav2:m9-mppi .
```

The optional image includes only Nav2 fixture packages, no robot
motor, depth camera or bridge source. Colcon/pytest and the installed
`mppi_shadow_probe` executable check run during `podman build`.
Do not replace or restart the running `leroy-ros`.

## Terminal 1 — isolated five-process M5 fixture

```bash
podman run --rm -it --name leroy-nav2-m9 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:m9-mppi bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration m5_controller_fixture.launch.py
  '
```

## Terminal 2 — manual lifecycle and one bounded synthetic goal

```bash
podman exec leroy-nav2-m9 bash -lc '
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
    echo "FAIL: motor-compatible topic detected" >&2
    exit 1
  fi

  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /controller_server activate

  ros2 run leroy_nav2_integration mppi_shadow_probe

  ros2 lifecycle set /controller_server deactivate
  ros2 lifecycle set /controller_server cleanup
  ros2 lifecycle set /controller_server shutdown
  ros2 lifecycle get /controller_server
'
```

Expected (the first labels may vary with startup order):

```text
M9 MPPI SHADOW: WAITING_MPPI_GRAPH_IDENTITY
M9 MPPI SHADOW: WOULD_VETO_REARM
M9 MPPI SHADOW: HEALTHY_DIAGNOSTIC_ONLY
PASS: M9 actual MPPI remapped diagnostic Twist observed during accepted SYNTHETIC FollowPath; ...
Transitioning successful
Transitioning successful
Transitioning successful
finalized [4]
```

If controller publisher discovery has already finished, the WAITING
line need not appear. If actual controller output arrives only
briefly, M9 may **FAIL** its 1s rearm or nonzero-output contract:
do not force the test to pass by loosening bounds or remapping
`/cmd_vel`.

After the probe PASS and `finalized [4]`, press Ctrl+C in Terminal 1.
Confirm all **five** fixture processes finish cleanly with no SIGSEGV,
especially `controller_server` with its embedded costmap. If the
probe fails, its `set -e` intentionally stops before lifecycle
cleanup; attempt manual controller deactivate/cleanup/shutdown if
feasible and then Ctrl+C. Collect both terminal logs.

## Acceptance and explicit nonclaims

**Pending operator validation:** Colcon build and pytest, actual
publisher and bounded nonzero diagnostic message, M7 policy rearm,
acknowledged cancellation/terminal action, graph isolation,
controller `finalized [4]`, and five clean fixture exits.

MPPI's diagnostic topic `geometry_msgs/Twist` has **no acquisition
timestamp or authenticity proof**; only receipt freshness can be
tested. Synthetic wheel motion is independent of MPPI commands, and
the test does not prove path tracking, goal completion, steering
safety, braking under missing depth/odom, TF protection,
obstacle avoidance, motor safety, any watchdog or live readiness.
M9 does not import `leroy_bridge`, access devices, start planner,
or publish `/cmd_vel`. No M10, hardware interface, or deployable
interlock is authorized by this milestone.

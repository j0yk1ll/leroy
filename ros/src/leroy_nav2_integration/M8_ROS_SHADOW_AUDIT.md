# Milestone 8 — Offline ROS-connected shadow policy audit

M7 proved only an in-process deterministic replay. M8 tests **wiring
that same pure M7 shadow policy to real ROS 2 messages from the M6
SYNTHETIC-only fixture**. It does not connect controller output to
motors, provide a drive interlock, or certify stopping behavior.

## Exact ROS graph boundary

The existing M6 fault fixture is included, unchanged:

- Fake mecanum odometry (or periodic fake odom+TF blackout)
- Fake 4×4 depth + CameraInfo (or periodic fake depth blackout)
- Depth-to-obstacle adapter (marking only, no clearing)
- Nav2 controller_server with embedded rolling `odom`-frame costmap
- Lifecycle manager with `autostart=false`

M8 adds a sixth fixture process:

- `synthetic_shadow_candidate`: publishes a **constant fictional
  `geometry_msgs/Twist`** at ~10Hz to **only**
  `/leroy_nav2/diagnostics/synthetic_shadow_candidate`.
  The message is NOT computed by MPPI, carries no timestamp, and has
  no robot/motor subscriber. No FollowPath goal is sent.

The operator separately starts `ros_shadow_audit`, a **read-only**
subscriber to the above fake candidate, M3's diagnostic Odometry,
M2's diagnostic obstacle PointCloud2 and the embedded `/local_costmap/costmap`.
It reuses the M7 `ShadowWindow` (which internally reuses M6's timestamp
freshness rules) and prints only terminal lines. It never publishes
a ROS topic, invokes a Nav2 action or issues a velocity command.

The M6 controller may advertise
`/leroy_nav2/diagnostics/proposed_cmd_vel` because M5 hard-remapped
its output, but **M8 does not subscribe to it**. The new fake topic
was deliberately named differently to make this distinction testable.

M8 accepts two independent sequences after a healthy baseline:

```text
HEALTHY_DIAGNOSTIC_ONLY
WOULD_VETO_BLOCKED_DEPTH       (or WOULD_VETO_BLOCKED_ODOM)
M8 SHADOW: expected fake source loss vetoed
WOULD_VETO_REARM
HEALTHY_DIAGNOSTIC_ONLY
PASS: M8 ROS shadow <mode> ... independent FICTIONAL Twist only ...
```

It must observe the correct veto continuously for at least 0.5s,
then show `WOULD_VETO_REARM` before positive health returns. Policy
recovery requires 1 second of continuously valid observations. The
probe also checks that the fake candidate topic's one publisher is
the `m8_synthetic_candidate` node, and fails on `/cmd_vel`, `/odom`,
or hardware/navigation bridge nodes. Deadline: 85 seconds. Both
ROS data and replay stimuli remain synthetic: no real sensor facts.

## Build (Arch Linux, no hardware or live ros container)

From the repository root:

```bash
git fetch origin
git switch feature/nav2-m8-offline-ros-shadow-audit
git pull --ff-only origin feature/nav2-m8-offline-ros-shadow-audit
cd ros
podman build -f Dockerfile.nav2 -t leroy-nav2:jazzy .
```

Image build runs Colcon and pytest across all five optional packages
and validates `synthetic_shadow_candidate` and `ros_shadow_audit`
executables exist.

## Trial A: fake depth blackout

Terminal 1:

```bash
podman run --rm -it --name leroy-nav2-m8 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration m8_ros_shadow_fixture.launch.py fault:=depth
  '
```

Terminal 2:

```bash
podman exec leroy-nav2-m8 bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  echo "=== INITIAL LIFECYCLE AND TOPIC SAFETY ==="
  state=""
  for i in $(seq 1 20); do
    state="$(ros2 lifecycle get /controller_server 2>/dev/null || true)"
    if [ "$state" = "unconfigured [1]" ]; then break; fi
    sleep 1
  done
  test "$state" = "unconfigured [1]"
  if ros2 topic list | grep -Eq "^(/cmd_vel|/odom)$"; then
    echo "FAIL: motor-compatible topic detected" >&2
    exit 1
  fi

  echo "=== ACTIVATE EMBEDDED LOCAL COSTMAP; NO FOLLOWPATH GOALS ==="
  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /controller_server activate

  echo "=== M8 READ-ONLY SHADOW OBSERVER ==="
  ros2 run leroy_nav2_integration ros_shadow_audit --ros-args -p fault_mode:=depth

  echo "=== CONTROLLER CLEANUP ==="
  ros2 lifecycle set /controller_server deactivate
  ros2 lifecycle set /controller_server cleanup
  ros2 lifecycle set /controller_server shutdown
  ros2 lifecycle get /controller_server
'
```

Expected: `PASS: M8 ROS shadow depth ...` and `finalized [4]`.
Afterward Ctrl+C in Terminal 1 and verify all **six** launched fixture
processes finish cleanly (M2 standalone costmap callback workaround
does NOT apply to the controller's embedded costmap; SIGSEGV fails M8).

If the audit fails, `set -e` deliberately leaves shutdown manual:
deactivate, cleanup and shutdown controller where possible, then
Ctrl+C, collect logs. Do not send a FollowPath goal or change the
non-clearing costmap merely to make the audit pass.

## Trial B: fake odometry blackout

Use a **fresh** disconnected container; repeat Terminal 1 replacing
`fault:=depth` with `fault:=odom`, and Terminal 2 replacing
`fault_mode:=depth` with `fault_mode:=odom`. Expect
`HEALTHY_DIAGNOSTIC_ONLY → WOULD_VETO_BLOCKED_ODOM →
WOULD_VETO_REARM → HEALTHY_DIAGNOSTIC_ONLY`, a PASS, `finalized [4]`
and six clean exits.

## Nonclaims / acceptance

- M8 **must** pass Colcon/pytest and both distinct synthetic ROS
  fault modes, including proof of no command-topic authority.
- The M7 positive health label is STILL **not** permission for the
  robot to move. This audit is not a command gate or watchdog.
- The candidate is fabricated and constant, NOT MPPI output; M8
  does not check whether MPPI proposes unsafe velocities during
  faults, brakes, or responds safely to missing sensors.
- The `Twist` is unstamped; receipt time cannot prove its origin.
- All wheel geometry, camera calibration, footprint, velocity limits,
  inflation and sensor coverage are fictional. The known synthetic
  inflation advisory (0.65m below ~0.721m circumscribed) remains.
- No hardware device/network mounts, live `leroy-ros`, real encoders,
  real cameras, XRSLAM, motor bridge, FollowPath action, `/cmd_vel`
  publisher, or physical robot interaction permitted.

Draft PR must remain unmerged until reviewed; no M9 or actual motor
interface without explicit separate authorization.

# M6 — disconnected sensor-loss / recovery diagnostic audit

**This is NOT a safety interlock, motor gate, or proof Nav2 will stop
when sensors disappear.** No hardware, robot networking, real sensors,
planner, FollowPath goals, or physical motion is connected.

## What M6 adds

M1–M5 verified the controller, synthetic depth, synthetic mecanum odom,
rolling local costmap and diagnostic-only MPPI action proposal. M6
exercises *negative* cases that successful nominal probes cannot test:

- **Depth blackout:** subclass the 4×4 fake camera fixture. Stop
  publishing both fake depth and CameraInfo during a 6 s gap.
  Normal synthetic odometry and dynamic TF continue.
- **Odometry blackout:** subclass the synthetic wheel fixture. Stop
  publishing Odometry and its dynamic `odom → base_link` TF for the same
  6 s gap. The fake camera/depth stream continues.
- **Recovery:** each source cycles through 10 s normal, 6 s stopped,
  6 s normal every 22 s. Stopped odom is **not extrapolated** across
  the gap; there is no empty clearing cloud.
- **Diagnostic health audit:** observes only timestamps/frames and the
  existing controller's marked-and-unknown local costmap. It requires
  `READY → BLOCKED_DEPTH/ODOM → READY` and rejects stale/future
  acquisition stamps even if a costmap remains published. It neither
  publishes a ROS topic nor controls Nav2 or velocity.

This demonstrates **only that the separate audit detects missing
synthetic data and subsequent recovery**. It does **not** prove the
controller/MPPI reacts safely to stale input. No `FollowPath` goal
is sent in M6, so MPPI is not commanded to move or emit new proposals.

## Build (Arch host)

Keep `leroy-ros` untouched and physical motors inhibited.

```bash
git fetch origin
git switch feature/nav2-m6-offline-fault-injection-audit
git pull --ff-only origin feature/nav2-m6-offline-fault-injection-audit
cd ros
podman build -f Dockerfile.nav2 -t leroy-nav2:jazzy .
```

The existing optional image build runs Colcon pytest and checks that
`fault_depth_fixture`, `fault_odom_fixture`, and `fault_audit` exist.

## Trial A — drop synthetic depth (two terminals)

Terminal 1:

```bash
podman run --rm -it --name leroy-nav2-m6 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration m6_fault_fixture.launch.py fault:=depth
  '
```

Terminal 2, while Terminal 1 is running:

```bash
podman exec leroy-nav2-m6 bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  echo "=== M6 DEPTH FAULT ==="
  state=""
  for i in $(seq 1 20); do
    state="$(ros2 lifecycle get /controller_server 2>/dev/null || true)"
    if [ "$state" = "unconfigured [1]" ]; then break; fi
    sleep 1
  done
  test "$state" = "unconfigured [1]"
  if ros2 topic list | grep -Eq "^(/cmd_vel|/odom)$"; then
    echo "FAIL: hardware-compatible topic" >&2; exit 1
  fi
  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /controller_server activate
  ros2 run leroy_nav2_integration fault_audit --ros-args -p fault_mode:=depth
  ros2 lifecycle set /controller_server deactivate
  ros2 lifecycle set /controller_server cleanup
  ros2 lifecycle set /controller_server shutdown
  ros2 lifecycle get /controller_server
'
```

Expected PASS:

```text
M6 AUDIT: READY
M6 AUDIT: BLOCKED_DEPTH
M6 AUDIT: expected synthetic sensor loss detected
M6 AUDIT: READY
PASS: M6 synthetic depth loss was diagnosed as stale, then recovered only on fresh sensor observations; diagnostic audit only, no /cmd_vel or live /odom, NO motor interlock claim
finalized [4]
```

The audit may print transient BLOCKED states during startup or
recovery; it must eventually prove the ordered transition with
a 0.5s persistent detection and within a 58s deadline.

After terminal 2 returns, Ctrl+C in terminal 1. Check that **all five**
processes finish cleanly, especially `controller_server` and its
embedded costmap, for which the M2 standalone crash workaround does
not apply. Do not count an exit -11 as a pass.

## Trial B — drop synthetic odometry

Start a fresh container with the same Terminal 1 command above but
change the launch argument to `fault:=odom`. In Terminal 2 repeat
the lifecycle/probe sequence using:

```bash
ros2 run leroy_nav2_integration fault_audit --ros-args -p fault_mode:=odom
```

Expected ordered audit transition is:
`READY → BLOCKED_ODOM → READY`.
Then deactivate/cleanup/finalize controller and Ctrl+C as in Trial A.

**Important:** do not attempt a second lifecycle configure after
`finalized [4]`; use an independent container for each mode.

## Acceptance / limitations

M6 is accepted for disconnected *audit-only* fault handling if the
optional image builds with all Colcon tests, both fault modes report
their ordered PASS, the ROS graph contains no `/cmd_vel` or `/odom`,
and both runs exit cleanly.

- Synthetic fault data are not real sensor dropout experiments.
- A rolling costmap can retain markings while the source is stale.
  Its existence does not make a path safe, nor prove freshness.
- No safety-critical command filter, E-stop, watchdog or motor gate was
  implemented. Audit status is a local printed test result only.
- The prior arbitrary square footprint's inflation warning
  (0.65 m versus circumscribed 0.721249 m) remains unmodified.
- No integration with Leroy's physical bridge, calibration, XRSLAM,
  motors or live `leroy-ros` container is authorized.
- Do not merge a draft PR or proceed to M7 without explicit review.

# Milestone 5 — Offline MPPI FollowPath diagnostic smoke test

**Not a closed-loop or real-robot controller; not collision avoidance.
Every velocity candidate stays in a disconnected synthetic ROS graph and
is published ONLY as an explicit diagnostic topic.** Never start this
fixture in the existing `leroy-ros` container or attach devices, robot
networking, controller bridge, motors or real sensors.

## Scope and architecture

Building on M1–M4, M5 uses synthetic mecanum odometry (M3), fake
camera/depth obstacle points (M2), and Nav2's local rolling obstacle
costmap (M4) inside an actual **MPPI controller_server** instance.

The controller has an *internal* `/local_costmap` node (not the M2
standalone `costmap_fixture`). M5's local costmap is in `odom` frame,
marks fake clouds, never raytrace-clears, and preserves unknown space.
MPPI's `Omni` model can output forward, sideways and angular candidates,
but its numeric limits and square footprint are still arbitrary
simulation parameters, **not Leroy calibration**.

The M3 odometry pattern is **independent of MPPI outputs**: this
experiment proves controller setup, action acceptance, bounded proposals
and remapping, NOT physical trajectory tracking or goal convergence.

All controller `cmd_vel` names are hard-remapped to:
`/leroy_nav2/diagnostics/proposed_cmd_vel`
(`geometry_msgs/msg/Twist`, Jazzy explicit
`enable_stamped_cmd_vel: false`). The diagnostic channel has **no
subscriber which can send motor commands**, and the test container has
no network or hardware access. Never publish or bridge it to `/cmd_vel`.

The probe constructs a short synthetic lateral `nav_msgs/Path` around
the current fake odometry pose and sends one `FollowPath` action goal.
It requires a marked+unknown `/local_costmap/costmap`, an accepted
goal, and at least one bounded nonzero diagnostic proposal within
9 seconds. It cancels the action on exit. This is not an autonomous
path completion test; if the MPPI costmap rejects the path (including
unknown or inflated obstacles), the test **fails** and operator logs
must be inspected rather than weakening safety configuration.

## Build (Arch Linux, from repository root)

```bash
git fetch origin
git switch feature/nav2-m5-offline-mppi-diagnostic-followpath
git pull --ff-only origin feature/nav2-m5-offline-mppi-diagnostic-followpath
cd ros
podman build -f Dockerfile.nav2 -t leroy-nav2:jazzy .
```

This optional image's build executes `colcon build`, `colcon test`,
`colcon test-result --verbose` for M1–M4 plus the new M5 tests in
`leroy_nav2_integration`. No live `ros/Dockerfile` is touched.

## Terminal 1 — disconnected synthetic-only controller graph

```bash
podman run --rm -it --name leroy-nav2-m5 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration m5_controller_fixture.launch.py
  '
```

The controller starts **UNCONFIGURED**. No goal runs automatically.

## Terminal 2 — lifecycle and one bounded synthetic action

```bash
podman exec leroy-nav2-m5 bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash

  echo "=== M5 GRAPH / SAFETY PRECHECK ==="
  ros2 node list
  if ros2 topic list | grep -Eq "^(/cmd_vel|/odom)$"; then
    echo "FAIL: live-compatible command or odom topic exists" >&2
    exit 1
  fi

  echo "=== M5 CONTROLLER LIFECYCLE ==="
  ros2 lifecycle get /controller_server
  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /controller_server activate

  echo "=== ACTION AND REMAP ACCEPTANCE ==="
  ros2 run leroy_nav2_integration controller_probe

  echo "=== CONTROLLED CLEANUP ==="
  ros2 lifecycle set /controller_server deactivate
  ros2 lifecycle set /controller_server cleanup
  ros2 lifecycle set /controller_server shutdown
  ros2 lifecycle get /controller_server
'
```

Expected probe success (not guaranteed until runtime tested):

```text
PASS: offline MPPI accepted synthetic FollowPath, produced bounded diagnostic-only velocity, controller costmap marked lethal+unknown, no /cmd_vel or live /odom; NO path completion claim
finalized [4]
```

If the probe fails, the `set -e` script intentionally stops before
cleanup. In that case do **not** send another action; manually
`ros2 lifecycle set /controller_server deactivate`, `cleanup`,
`shutdown` if possible, then Ctrl+C in Terminal 1 and paste logs.
Never force a failing test to PASS by deleting cost/obstacle critics
or enabling unknown-space clearing.

Once lifecycle is finalized, Ctrl+C in Terminal 1 and verify clean
exits. **Attention:** embedded Nav2 costmaps might still have the
Jazzy callback-group destructor crash seen in standalone Nav2 in M2.
The M2 workaround was specific to `costmap_fixture`; it does NOT
patch the controller_server's embedded costmap. Any SIGSEGV means M5
teardown has FAILED and requires isolated diagnostics.

## Acceptance criteria and limitations

- Optional-image Colcon build/test success; M5 source contract tests pass.
- Controller `configure/activate` succeeds with fake odom and
  non-clearing marked+unknown internal costmap.
- Synthetic `FollowPath` action accepted, at least one bounded,
  finite, planar **nonzero** diagnostic `Twist` observed.
- No ROS `/cmd_vel` or normal `/odom`, no bridge, hardware or motors.
- Action cancellation, lifecycle cleanup/finalization and clean Ctrl+C
  shutdown, with no segmentation fault.

A single finite accepted FollowPath **does not** demonstrate real path
tracking, collision avoidance, robustness to wheel slip, command
security outside the disconnected container, or readiness for physical
movement. Keep PR draft pending actual operator Podman runtime logs.
No M6, real sensing, XRSLAM, or motor interface authorized.

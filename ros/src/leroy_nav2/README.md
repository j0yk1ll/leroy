# Leroy Nav2 — HISTORICAL Milestone 1: isolated inspection

**Historical test instructions, not the active robot startup path.**
See [`ros/CURRENT_LIVE_RUNTIME.md`](../../CURRENT_LIVE_RUNTIME.md)
for the operator-verified Stage 3/4C system and current migration work.

**Not a driving stack.** The original M1 isolated Nav2 experiment did
not modify the then-existing hardware bringup, bridge or camera stack.
Its historical warning about a bridge `/cmd_vel` subscriber is **stale**:
the current `leroy_bridge` source deliberately has **no raw
`/cmd_vel` subscription** and its optional signed motion ingress is
not provisioned by Stage 3. Never join these old experimental
controller graphs to a physical motor authority.

## Exactly what runs

An optional `leroy-nav2:jazzy` image is independently built from
`osrf/ros:jazzy-desktop`, not from the expensive hardware/DepthART image.
Only `leroy_nav2` is copied into it. The dry-run launch creates a
`nav2_controller/controller_server` and `nav2_lifecycle_manager`.
Lifecycle `autostart: false`: the controller remains UNCONFIGURED.
No planner, navigation actions, recovery behaviors, odometry publisher,
dynamic TF publisher, fake scan, camera, motor bridge or motor gate runs.

The controller's both relative and absolute `cmd_vel` outputs are
hard-remapped to `/leroy_nav2/diagnostics/proposed_cmd_vel`. That topic
is diagnostic, **not** a hardware `Twist` interface. There is no subscriber
and no code connecting it to physical `/cmd_vel`.

For M1 the strongest boundary is `podman run --network=none` (no
network access), **no device mapping**, and **no host/network namespace
sharing**. A dedicated `ROS_DOMAIN_ID=167`, localhost DDS discovery and
a launch environment guard are defense in depth, not proof of isolation.
The image does not start Nav2 automatically.

## Build (Arch host shell, rootless Podman)

Keep physical motor power inhibited and E-stop available; leave the
running `leroy-ros` container untouched. In your checked-out clone:

```bash
git fetch origin
git switch feature/nav2-m1-isolated-dry-run
cd ros
podman ps --filter name=leroy-ros
podman build --file Dockerfile.nav2 --tag leroy-nav2:jazzy .
```

Build itself runs `colcon build`, `colcon test` and
`colcon test-result --verbose`. If a step fails, the image build fails.

## Start the disconnected diagnostic graph (Arch host shell)

```bash
podman run --rm -it --name leroy-nav2-dry-run \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; ros2 launch leroy_nav2 dry_run.launch.py'
```

No `--network=host`, device, serial, `--privileged`, or mounts.
In a SECOND Arch terminal, inspect *this test container only*:

```bash
podman exec leroy-nav2-dry-run bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  echo "=== installed Jazzy/Nav2 packages ==="
  dpkg-query -W "ros-jazzy-nav2-controller" "ros-jazzy-nav2-mppi-controller" "ros-jazzy-nav2-costmap-2d"
  ros2 pkg executables nav2_controller
  ros2 pkg executables nav2_lifecycle_manager
  echo "=== controller lifecycle (expect unconfigured) ==="
  ros2 lifecycle get /controller_server
  echo "=== nodes and topics ==="
  ros2 node list
  ros2 topic list
  echo "=== controller interfaces; NO published /cmd_vel allowed ==="
  ros2 node info /controller_server
  echo "=== prohibited /cmd_vel topic check ==="
  if ros2 topic list | grep -qx /cmd_vel; then
    echo "FAIL: hardware command topic exists in dry run" >&2
    exit 1
  fi
  echo "PASS: /cmd_vel absent in disconnected test graph"
'
```

Lifecycle service introspection may take a few seconds after launch.
Do NOT configure or activate the controller in M1; no valid `odom ->
base_link` chain exists. Do NOT send FollowPath goals. To exit, Ctrl-C
in the run terminal. Confirm hardware container was not changed:
`podman ps --filter name=leroy-ros`.

## Test only (Arch host shell)

```bash
podman run --rm --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    cd /leroy_ws
    colcon test --packages-select leroy_nav2 --event-handlers console_direct+
    colcon test-result --verbose
  '
```

For an optional RViz display later, use
`ros/src/leroy_nav2/rviz/dry_run.rviz`. M1 deliberately publishes no
odometry, TF, or sensor observations, so the costmap cannot yet be
validated visually. Never weaken network isolation merely to show RViz.

## Safety and validation status

* The 1 m square local costmap footprint is an arbitrary **simulation
  placeholder**, not a measured Leroy footprint. Wheel-center spacing
  cannot establish the robot's outer boundary.
* Software velocity/acceleration limits in `simulation.yaml` are
  **illustrative only**, unrelated to the bridge's open-loop PWM or
  experimentally validated stopping distance.
* `config/hardware.disabled.yaml` is an explicit deployment-policy
  manifest, **not** ROS parameter input. The launch rejects
  `mode:=passive_live` and `mode:=hardware`.
* Sensor observations have no marking/clearing source yet: unknown is
  NOT declared free by a fictitious camera, scan or odometry stream.
* The environment check cannot detect unsafe Podman network flags:
  verify the command line and actual endpoints. No automatic enabling
  of `/cmd_vel` exists.

Milestone-1 pass requires a successful new image build, passing colcon
tests, a running but unconfigured controller, no `/cmd_vel` topic,
and unchanged `leroy-ros`. These are **operator validation steps**;
a source review alone cannot establish those runtime facts. Stop here
and provide the actual logs for review before Milestone 2.

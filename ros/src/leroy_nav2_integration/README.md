# Milestone 4 — disconnected odometry + depth → rolling costmap

**Offline synthetic integration only.** M4 exercises Nav2's odom-centered,
rolling, obstacle-only costmap using M3's fictitious mecanum odometry
and M2's 4x4 fake depth. The camera extrinsics, footprint, wheel geometry,
odometry covariance, timing, and motion pattern are all SYNTHETIC,
not measured Leroy performance. This is neither navigation nor a
collision-prevention/safety system.

## Synthetic graph

```text
synthetic_odom → /leroy_nav2/diagnostics/synthetic_odom  [Odometry]
            └→ /tf: odom → base_link
synthetic_depth_fixture → /tf_static: base_link → sim_camera_optical_frame
                       → /depthart/depth + /camera/camera_info
depth_obstacles → raw cloud (camera optical) + obstacle cloud (base_link)
             └→ costmap_fixture in odom frame → /costmap [OccupancyGrid]
```

The costmap uses a rolling 4m×4m window, ROS2 integer width/height,
marking-only PointCloud2 obstacles, **no raytrace/free-space clearing**
and a wholly arbitrary 1m-square footprint with a synthetic inflation
radius. Unknown cells must remain unknown. No map→odom TF or localization
is fabricated; a moving odom-centered local window is not a global map.

There is **no controller, planner, navigator, FollowPath action, velocity
producer, motor bridge, live sensor driver, or hardware ROS network.**
The already-tested M2 `costmap_fixture` workaround is retained for
callback-group teardown, but not asserted to fix upstream Nav2.

## Build on Arch Linux, rootless Podman

Leave `leroy-ros` running and untouched; ensure motors are physically
inhibited. From the repository root:

```bash
git fetch origin
git switch feature/nav2-m4-offline-odom-depth-rolling-costmap
git pull --ff-only origin feature/nav2-m4-offline-odom-depth-rolling-costmap
cd ros
podman build -f Dockerfile.nav2 -t leroy-nav2:jazzy .
```

The build runs Colcon with pytest and rejects test failures before
installing an image. Hardware bridge, XRSLAM, DepthART real driver,
motors, and host device access are excluded.

## Terminal 1 — start synthetic graph, no networking or host devices

```bash
podman run --rm -it --name leroy-nav2-m4 \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  leroy-nav2:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    ros2 launch leroy_nav2_integration integrated_fixture.launch.py
  '
```

## Terminal 2 — configure/activate costmap and probe

```bash
podman exec leroy-nav2-m4 bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  echo "=== COSTMAP LIFECYCLE ==="
  ros2 lifecycle get /costmap
  ros2 lifecycle set /costmap configure
  ros2 lifecycle set /costmap activate
  echo "=== SYNTHETIC ROLLING COSTMAP PROBE ==="
  ros2 run leroy_nav2_integration integrated_probe
  echo "=== CONTROLLED SHUTDOWN ==="
  ros2 lifecycle set /costmap deactivate
  ros2 lifecycle set /costmap cleanup
  ros2 lifecycle set /costmap shutdown
  ros2 lifecycle get /costmap
'
```

Expected result:

```text
PASS: synthetic odom->base_link->camera TF, same-stamp depth clouds, odom-frame lethal+unknown rolling costmap tracks forward/lateral motion; no /cmd_vel or /odom
finalized [4]
```

The finite probe also requires a latched `base_link → sim_camera_optical_frame`
static TF with a valid quaternion and positive synthetic camera height.
It monitors stamped odometry and TF, both synthetic cloud
frames and matching acquisition stamps, moving x/y/yaw poses, lethal
and unknown space together in the same published costmap, rolling
origin shifts for forward and sideways movements, and window center
following the robot within 0.35m. It fails if `/cmd_vel`, `/odom`,
controller/planner/navigator/bridge nodes appear. Probe deadline 28s;
it needs a complete fake-wheel movement cycle.

After the controlled shutdown, press Ctrl+C in Terminal 1 and record
that all five processes (including `costmap_fixture`) exit cleanly.

## Acceptance gates

* Image Colcon build/test passes for **all five** optional packages.
* ROS probe PASS with costmap `global_frame=odom`, observed rolling
  origin motion in **both x and y**, unknown and lethal cells together.
* `finalized [4]` lifecycle and clean Ctrl+C with no SIGSEGV.
* `/cmd_vel`, real `/odom` and controller processes must be absent.
* Strict network-none Podman, no mounts/devices/privileges; no interaction
  with `leroy-ros`.

M4 does **not** validate wheel encoders, TF extrinsics, real depth quality,
physical traction, stopping distance, obstacle coverage, map localization,
controller output, or any real collision safety. No live robot permission.
Do not merge this PR, enable hardware, or proceed to M5 without explicit
review/approval.

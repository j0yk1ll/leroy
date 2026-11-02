# Leroy map-aware navigation (non-actuating)

The controller accepts `/leroy/navigation/goal` (PoseStamped in `odom`), consumes `/odom` and `/leroy/navigation/occupancy_grid` (nav_msgs/OccupancyGrid in `odom`), and outputs `/leroy/navigation/cmd_vel` only. It is not wired to the MCU or safety signer.

The new A* route planner treats unknown occupancy values as obstacles, inflates occupied cells by a configurable footprint radius, rejects diagonal corner cutting and periodically replans around changes. Missing, stale or unusable maps yield zero velocity requests.

**Required:** a map producer supplying a metric, appropriately framed and updated occupancy grid, plus calibrated /odom. This PR does not yet implement map generation, full Nav2, lifecycle management, dynamic obstacle prediction or a certified collision monitor. Inflated grid planning is not a physical safety claim.

The grid origin must have zero yaw; nonzero map-origin yaw is rejected. Footprint inflation defaults to 0.30 m and is provisional. Update the geometry only after measuring Leroy. Use replay/simulation before physical integration.

Run pure unit tests from `ros/` after sourcing the workspace:

```bash
python3 -m unittest discover -s tests/unit -p 'test_grid_planner.py' -v
python3 -m unittest discover -s tests/unit -p 'test_goal_controller.py' -v
```

## Depth-to-occupancy mapping

`ros2 run leroy_navigation depth_grid` consumes `/perception/raw_depth_points` and publishes the planner's `/leroy/navigation/occupancy_grid` in the `odom` frame. Every frame is rebuilt from transformed depth rays; measured traversals are free (0), obstacle-height endpoints occupied (100), and unobserved cells unknown (-1). This is a **rolling local snapshot**, not SLAM or a persistent global map. Map origin stays unrotated as the planner requires.

Mapping starts disabled until `metric_depth_calibrated`, `extrinsics_calibrated` and `ground_datum_calibrated` are set true by a separately performed calibration procedure. These parameters are configuration assertions, not independent proofs or safety interlocks. Do not enable them for unvalidated DepthART depth estimates. Ground datum assumes odom z=0 at the ground plane. Camera/robot transforms must be available at the point cloud timestamp.

The map only marks sensed rays free and cannot see behind obstacles or outside the camera FOV. The A* planner regards unknown cells as blocked. A forward monocular sensor cannot deliver all-around navigability; additional observation coverage and map persistence or mapping localization will be necessary for broader navigation. No motors are connected to this package.

## Rolling map accumulation

The mapper now accumulates grid-cell observations in a world-aligned odom window. Each observation lasts at most `evidence_lifetime_s` (default 3 seconds); later directly observed freespace can replace an old occupied observation. Moving-window cells retain their world coordinates and observations outside the window are evicted. A timestamp regression resets the map. Frames with no usable cloud are not published, so the planner's own map timeout inhibits navigation. This is short-horizon accumulation, **not** permanent SLAM mapping or loop-closure correction.

## Waypoint missions

Send a `nav_msgs/Path` in the `odom` frame to `/leroy/navigation/mission`; each `PoseStamped` becomes an ordered goal. The controller advances to the next pose after position and final heading tolerances are reached. A new mission replaces the old one. `/leroy/navigation/goal` remains the single-goal interface (also replaces the current mission). Publish `std_msgs/Empty` to `/leroy/navigation/cancel` to discard all remaining goals and request zero velocity on the next controller tick. The functional completion result is published as `std_msgs/String` on `/leroy/navigation/mission_result`: `SUCCEEDED`, `CANCELLED` or `FAILED_UNREACHABLE`.

An unavailable route is retried at the regular planning interval up to three times, then the mission fails and stops. Missing or stale odometry/map continues to hold zero output rather than falsely declaring completion. This is mission orchestration on the existing map-aware controller, not Nav2 Actions or motor authorization.

## Nav2 backend (ROS 2 Jazzy)

Run **one** navigation backend at a time. Native Nav2 provides `NavigateToPose`, `NavigateThroughPoses`, `FollowWaypoints`, A* global planning via Navfn, holonomic DWB local control, recovery behavior actions and lifecycle bringup. The custom `goal_controller` remains an alternative experimental backend and must not be started simultaneously.

```bash
colcon build --packages-up-to leroy_navigation
source install/setup.bash
ros2 launch leroy_navigation nav2.launch.py use_sim_time:=false
```

The producer `depth_grid` must independently provide `/leroy/navigation/occupancy_grid` with validated metric depth, ground height, camera extrinsics and `odom` TF. Nav2 costmaps use that grid in the **odom** global frame, so they need no synthetic `map->odom` transform. This is local navigation within the accumulated grid, not loop-closed global navigation. The map is short-lived and may prevent planning to unknown destinations until observed.

Nav2 emits only `/leroy/navigation/nav2_cmd_vel` (Twist). It is intentionally not connected to `/cmd_vel`, `/leroy/navigation/cmd_vel`, the safety signer or STM32. A future safety commissioning step must explicitly integrate motion requests. All robot geometry, speed and costmap numbers in `config/nav2.yaml` are **provisional** and must be measured and simulation-tested before physical deployment.

Example native action goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
  '{pose: {header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

**Integration limits:** Lifecycle startup needs a functioning occupancy grid and odometry/TF. A sparse front-camera map will typically block unknown routes. This branch has not been built or driven in simulation yet; do not interpret Nav2 configuration as physical autonomous readiness.

## Live navigation visualization (RViz2)

Start Nav2 and its odometry/map sources, then start the *read-only* visualization:

```bash
ros2 launch leroy_navigation visualization.launch.py \\
  image_topic:=/camera/image_raw use_features:=true
```

The dashboard displays raw camera images, a separately derived OpenCV Shi–Tomasi feature overlay, odom-based robot pose/TF, rolling occupancy grid, optional Nav2 local and global costmaps, the live global planner route (`/plan`) and the selected goal marker (`/goal_pose`). RViz subscribes continuously and refreshes when ROS publishes new data; it has no motor/safety signer connection. The camera image topic is configurable at launch. Map and route require actual publishers, and RViz will show missing-topic warnings rather than fabricated content.

**XRSLAM's own feature tracker and 3D map are not exported by the Leroy ROS adapter.** Its existing PC visualizer remains the accurate source of those internal tracks; the OpenCV overlay is *not* claimed to show XRSLAM features. The map shown here is the Leroy rolling occupancy grid, not XRSLAM's internal sparse visual map. Connecting that sparse map requires an explicit XRSLAM export interface in a future change.

Nav2's `/plan` and `/goal_pose` are standard visualization topics; if a specific Nav2 deployment namespaces these topics, update the RViz display subscriptions accordingly. The `2D Goal Pose` RViz tool publishes to `/goal_pose`, not the motor bridge. This visualization does not modify localization or safety behavior.

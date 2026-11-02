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

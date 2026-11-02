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

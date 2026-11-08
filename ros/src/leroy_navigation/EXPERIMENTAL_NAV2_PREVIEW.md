# Stage B1: Experimental voxel-to-Nav2 occupancy preview

**This is a map interface prototype, NOT an authorized Nav2 costmap.**
The active Stage A8 cloud is projected into a 2D OccupancyGrid using a
configurable **uncalibrated** world-Z height slab. Every observed cell is
marked occupied (100); every unobserved cell is unknown (-1). No free-space
ray tracing, calibration, height-datum verification, or robot footprint
clearance is implied.

## Bring-up

Run camera, VDA, XRSLAM experimental metric preview, experimental scene
cloud, and voxel mapper as before, then:

```bash
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
ros2 launch leroy_navigation experimental_nav2_preview.launch.py
```

The node publishes:

- `/xrslam/experimental_navigation/occupancy_preview` — occupied-only
  `nav_msgs/OccupancyGrid`, transient-local, reliable
- `/xrslam/experimental_navigation/preview_status` — readiness, metric
  segment, occupied-cell count and error flags

Verify:

```bash
timeout 5s ros2 topic echo /xrslam/experimental_navigation/preview_status \
  std_msgs/msg/String --qos-durability transient_local --once --field data
timeout 5s ros2 topic echo /xrslam/experimental_navigation/occupancy_preview \
  nav_msgs/msg/OccupancyGrid --qos-durability transient_local --once --field info.width
```

In RViz, set Fixed Frame manually to
`xrslam_experimental_metric_world`. Add a Map display on
`/xrslam/experimental_navigation/occupancy_preview`, Reliability
**Reliable**, Durability **Transient Local**. Inspect the map alongside
the active 3D cloud.

**Do not** remap this topic to `/leroy/navigation/occupancy_grid`.
The existing Nav2 costmap is framed in `odom` and requires independently
validated metric depth, camera extrinsics, ground datum and odometry.
Publishing unvalidated experimental voxels into that stack would conceal
scale and frame discontinuities. The preview intentionally publishes no
TF, odometry, `cmd_vel`, costmap clearings or navigation goals.

On a metric-scale segment change the node invalidates the prior grid
with a one-cell unknown map, and waits for new current-segment clouds.
On temporary status suspension, it retains its last view but accepts
no new clouds. The preview is *not* archived across segments.

This B1 slice validates the map message interface and obstacle-only
projection. Real Nav2 planning and closed-loop driving are separate
milestones requiring calibrated, coherent navigation inputs.

# Stage A7: Experimental session-local voxel map

This map is an **RViz visualization only**. It is not metric-certified,
registered to ROS navigation TF, an obstacle grid, a free-space source or
an authorization to move the robot. Keep `pose_convention_validated=false`
and `enable_map_tf=false` with native `depth_constraints_added=0`.

## Inputs

- `/xrslam/experimental_scene/depth_cloud` optical-frame 3D scene cloud
- `/xrslam/experimental_metric/pose` XRSLAM prototype scaled body pose
- `/xrslam/experimental_metric/status` latched scale segment/state
- Static body<-optical camera transform from the URDF TF tree

The implementation matches **acquisition timestamps**, not arrival times.
No camera-to-world extrapolation or identity TF substitute is permitted.
The adapter resolves the static `base_link <- camera_optical_frame` transform
before using the unverified XRSLAM body pose as a visualization-only transform.
If `XRSLAM_RESULT_BODY_POSE` later proves to use a different convention, this
must be corrected before physical interpretation of the map.

The map uses 5 cm voxel cells with a FIFO cap of 60,000 cells, retains one
representative point per cell, and renders a bounded PointCloud2 snapshot once
per second. A new metric scale segment clears the previous cloud immediately.
A suspended metric preview pauses accumulation. The existing raw camera cloud
and unscaled XRSLAM trajectory are not affected.

## Run

With camera, rectifier, Metric Video Depth Anything Small, XRSLAM experimental
metric preview, and the Stage A6 scene-cloud adapter all running:

```bash
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
ros2 launch leroy_perception experimental_voxel_map.launch.py
```

In another terminal:

```bash
timeout 10s ros2 topic echo /xrslam/experimental_scene/voxel_map_status \
  std_msgs/msg/String --qos-durability transient_local --once --field data
timeout 10s ros2 topic echo /xrslam/experimental_scene/voxel_map \
  sensor_msgs/msg/PointCloud2 --once --field width
```

In RViz set the Fixed Frame to `xrslam_experimental_metric_world`,
then add a `PointCloud2` display with topic
`/xrslam/experimental_scene/voxel_map`. The experimental world is isolated
from ROS navigation TF by design; do not use the robot's `map` frame.

## Diagnostics

The status includes the current metric `segment_id`, `active` flag, voxel
count, cloud accepted/rejected/unmatched counts, and `last_error`.

- `voxels > 0`: accumulation started, **not** proof of correct geometry.
- `unmatched_clouds` rises: camera/depth/pose timestamps do not align
  within `max_pose_delta_s`; inspect stamps and delays first.
- `last_error=LookupException...`: the static camera-to-body TF is absent,
  or `camera_frame` does not match the depth cloud header.
- `state=suspended_...`: stop accumulating until the metric preview resumes.
- A new segment clears the accumulated cloud; no cross-segment merging.

No Nav2 subscriptions, motor outputs, costmap clearings, or global TF are
created by this adapter. Tune voxel size and pose-match tolerance only after
inspecting observed frame/time behavior.

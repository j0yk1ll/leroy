# Experimental scene cloud (Stage A6)

Assumes Metric Video Depth Anything Small works as a **prototype** depth
backend; does not claim verified metric ranging, ground segmentation,
occupancy, free space or navigation safety.

The `video_depth_scene.launch.py` launch starts only the existing guarded
`depth_obstacles` adapter, configured to publish a sampled point cloud
from `/video_depth_anything/depth`:

- Input: `/video_depth_anything/depth` (`sensor_msgs/Image`, 32FC1)
- Calibration: `/camera/camera_info` (exact timestamp, rectified P)
- Output: `/xrslam/experimental_scene/depth_cloud` (`sensor_msgs/PointCloud2`)
- Frame: original camera optical frame, not a global map
- Stride: 8 (up to 4,800 points for a 640x480 image)
- Obstacle output **disabled** and TF/ground authority **false**

## Run

With the camera, rectifier and Metric VDA depth node already running:

```bash
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
ros2 launch leroy_perception video_depth_scene.launch.py
```

In RViz set Fixed Frame to the depth image's camera optical frame, add
PointCloud2 for `/xrslam/experimental_scene/depth_cloud`, and inspect scene
structure. This cloud follows the camera; it is **not** an accumulated world
model. The separate existing XRSLAM experimental pose and sparse map remain
available for visual trajectory inspection.

To check a single cloud without waiting indefinitely:

```bash
timeout 10s ros2 topic echo /xrslam/experimental_scene/depth_cloud \
  sensor_msgs/msg/PointCloud2 --once --field width
```

If no cloud appears, inspect the adapter logs. The existing adapter rejects
non-matching CameraInfo timestamps, stale predictions, bad rectified
intrinsics and invalid depth samples. Do not work around those validation
checks by fabricating timestamps or TF.

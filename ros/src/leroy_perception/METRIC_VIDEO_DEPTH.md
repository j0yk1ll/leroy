# Experimental Metric Video Depth Anything Small (Stage A5)

This is a prototype-only second depth backend; it does **not** provide
validated navigation depth or pose. The original DepthART backend remains
available. Do not enable `pose_convention_validated`, `enable_map_tf`, or native
depth constraints on the basis of these predictions.

## External upstream installation in `leroy-viz`

The upstream repository is **not vendored** in Leroy. Follow its installation
requirements for your installed PyTorch/torchvision release:

```bash
cd /opt
git clone https://github.com/DepthAnything/Video-Depth-Anything.git
cd Video-Depth-Anything
python3 -m pip install -r requirements.txt
mkdir -p checkpoints
curl -fL --retry 3 \
  -o checkpoints/metric_video_depth_anything_vits.pth \
  https://huggingface.co/depth-anything/Metric-Video-Depth-Anything-Small/resolve/main/metric_video_depth_anything_vits.pth
sha256sum checkpoints/metric_video_depth_anything_vits.pth
```

Expected checkpoint SHA-256 from the official Hugging Face model file:
`3c28432b4e1f0d7bb31cad5151b6313b49457db5aa58d82e85bfb0f8b1311b33`.
The model file is roughly 116 MB. Avoid running pip as root into an
externally-managed ROS Python environment if prohibited; use an environment
with compatible ROS 2 Python bindings instead. Ensure the same interpreter
can import `rclpy`, `cv_bridge`, `torch`, `torchvision`, and `cv2`.

The external code and model may change. Pin a validated upstream commit for
reproducible experiments. This adapter loads the official Small metric
weights strictly; it will fail loudly if the streaming code is incompatible.

## ROS build and run

```bash
cd /leroy_ws
colcon build --packages-select leroy_perception
source install/setup.bash
ros2 launch leroy_perception metric_video_depth.launch.py device:=cpu
```

The default input is `/camera/image_rect`; output is
`/video_depth_anything/depth` (`sensor_msgs/Image`, `32FC1`).
For CPU, `input_size=280` is an experiment, not a guaranteed FPS.
The underlying model is designed for temporally consistent video depth; on
CPU, streaming inference may be slow. The wrapper uses a latest-frame-only
queue and retains the exact image header for asynchronous matching.

Start the XRSLAM adapter separately with these *additional* parameters
in addition to the existing calibration and A1 safety-gated options:

```text
-p depth_topic:=/video_depth_anything/depth
-p async_depth_stage_a1:=true
-p experimental_metric_preview:=true
-p pose_convention_validated:=false
-p enable_map_tf:=false
```

Do **not** simultaneously run DepthART on the same topic. To compare
backends, use separate controlled runs, or run DepthART on its normal
`/depthart/depth` topic while selecting only one XRSLAM depth_topic.

## Inspect

```bash
ros2 topic echo /video_depth_anything/status --qos-durability transient_local --once
ros2 topic hz /video_depth_anything/depth
ros2 topic echo /xrslam/depth_sync_status std_msgs/msg/String --once --field data
ros2 topic echo /xrslam/experimental_metric/status --qos-durability transient_local --once
```

Important checks: `candidate_scale_resets`, `candidate_window_relative_mad`,
`geometry_frames_matched`, depth association rate, and physically measured
translation. Compare them against DepthART using equivalent movement, scene,
and camera settings. Do not interpret low scale-window MAD as proof of
absolute metric accuracy.

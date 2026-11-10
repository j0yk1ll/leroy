# Relative-depth backend comparison — VDA vs ZipDepth

Status: **experimental, source-reviewed, not deployed or measured on Leroy**.
This feature branch is based on `fix/ros-tof-stream-shutdown`. It does not
change MCU firmware, actuator code, XRSLAM's estimator, or the default robot launch.

## Source references (reviewed November 2026)

- [VDA upstream](https://github.com/DepthAnything/Video-Depth-Anything):
  Video Depth Anything Small, 28.4M parameters, video-aware temporal streaming;
  relative checkpoint `video_depth_anything_vits.pth`; Small license Apache-2.0.
  Streaming is experimental, and the authors report reduced accuracy versus
  offline video inference.
- [ZipDepth upstream](https://github.com/fabiotosi92/ZipDepth):
  ZipDepth Base, approximately 6.1M fused parameters, 2026 ECCV project;
  MIT-licensed source, `zipdepth_base_npu.pth` with unfold-free upsampling;
  per-frame inference, not a temporal video model. Exports ONNX/TorchScript.
  Reported GPU figures are **not** benchmarks on Leroy hardware.
- Leroy's existing `metric_video_depth_node.py` and
  `tof_scale_{node,core}.py` stay unchanged in this initial comparison.

## Opt-in ZipDepth ROS adapter

`zipdepth_relative_node.py` consumes `/camera/image_rect` and publishes
`/relative_depth/zipdepth` (`sensor_msgs/Image`, `32FC1`) with the
**original camera header**. A separate string status stream is published on
`/relative_depth/zipdepth_status`. Latest-frame-only buffering avoids
infinite queues under CPU load. The external upstream checkout and its
checkpoint must be installed; nothing is downloaded automatically.

The default launch is **not included in** `robot.launch.py`:

```bash
ros2 launch leroy_perception zipdepth_relative.launch.py \
  python_executable:=/path/to/python-with-rclpy-torch-zipdepth
```

Defaults: `model_root=/opt/ZipDepth`, external NPU-compatible checkpoint,
CPU, input size 384. The adapter calls upstream `DepthInference.infer_image()`
on BGR frames and does not scale, clip to metric thresholds, feed actuators,
or publish to the old metric-depth topic. Pin and verify a specific upstream
ZipDepth commit/checkpoint before a reproducible build.

No launch command in this document authorizes real robot movement.

## Depth calibration is a separate problem

**Neither relative model predicts verified meters.** ZipDepth's documented
training uses a *scale-and-shift invariant* loss; one ToF observation
cannot independently solve both affine scale and shift, even when the
ToF return and camera surface correspond. Do not simply feed ZipDepth output
to `scale_relative_depth()` and set
`linear_relative_depth_validated=true`.

For VDA Relative, likewise verify its numerical output semantics before
assuming that multiplying by one ToF-derived scale produces physical Z.
ToF measurements are finite-beam range observations, not pixel-wise Z.
Camera/ToF 6-DoF extrinsics, range-to-camera Z projection, synchronized
acquisition timestamps, occlusions, and ROI surface homogeneity all matter.
The current STM32 sensor-only bridge gives ROS receipt timestamps, not
hardware-synchronized exposure timestamps.

Existing `tof_scale_node.py` is therefore retained diagnostic-only.
The depth-to-grid `metric_depth_calibrated` flag and physical movement
authorization **must remain disabled**.

## Decision and acceptance plan

1. Keep existing VDA Metric as the known experimental baseline; develop
   **VDA Relative Small** as the first metricization target, because the
   streaming integration already exists. Add ZipDepth as a selectable
   relative-only backend, not a simultaneous autonomous source.
2. Capture the same calibrated camera sequence and ToF reference targets;
   compare CPU end-to-end latency (including ROS serialization), skipped
   frames, thermal load, temporal jitter, edge consistency, and depth ranking.
3. Validate prediction semantics on moving **and static** targets. Use
   multiple physically measured ranges and geometric correspondences to
   fit/test any scale+shift transformation; with only one ToF anchor,
   maintain explicit ambiguity rather than certifying absolute depth.
4. Test range gating under target changes, ToF dropout, clock offset, camera
   motion, and heterogeneous ToF cone observations; missing anchors must
   yield invalid metric output, never stale-but-plausible depth.
5. Only after measured validation, publish a separately named registered
   metric-Z depth stream and configure XRSLAM semantic output and the
   obstacle adapter to consume it. Unobserved space must not be cleared by
   absence of points. Keep Nav2 and motor authority inhibited pending
   independent geometry and stopping acceptance.

This initial ZipDepth adapter is source-staged only. It has **not yet passed**
a Leroy ROS build, inference test, calibration, or live hardware validation.

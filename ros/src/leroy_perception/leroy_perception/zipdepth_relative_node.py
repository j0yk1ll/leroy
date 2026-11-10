#!/usr/bin/env python3
"""Opt-in ZipDepth relative-depth ROS adapter.

Publishes raw, uncalibrated model predictions ONLY. Values are NOT meters,
inverse meters, occupancy evidence, or a motion-authorization signal.
This deliberately does not publish to /video_depth_anything/depth.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ZipDepthRelativeNode(Node):
    def __init__(self):
        super().__init__("zipdepth_relative")
        self.declare_parameter("image_topic", "/camera/image_rect")
        self.declare_parameter("relative_depth_topic", "/relative_depth/zipdepth")
        self.declare_parameter("status_topic", "/relative_depth/zipdepth_status")
        self.declare_parameter("model_root", "/opt/ZipDepth")
        self.declare_parameter("checkpoint", "checkpoints/zipdepth_base_npu.pth")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("input_size", 384)
        self.declare_parameter("torch_threads", 4)
        self.declare_parameter("npu_upsampling", True)
        self.declare_parameter("stats_period_sec", 5.0)
        p = lambda name: self.get_parameter(name).value
        image_topic = str(p("image_topic"))
        depth_topic = str(p("relative_depth_topic"))
        status_topic = str(p("status_topic"))
        root = Path(str(p("model_root"))).expanduser().resolve()
        checkpoint = Path(str(p("checkpoint"))).expanduser()
        if not checkpoint.is_absolute():
            checkpoint = root / checkpoint
        device = str(p("device")).lower()
        input_size = int(p("input_size"))
        threads = int(p("torch_threads"))
        self._period = float(p("stats_period_sec"))
        upsample_npu = bool(p("npu_upsampling"))
        if (device not in ("cpu", "cuda") or not 128 <= input_size <= 1024
                or not 1 <= threads <= 64
                or not math.isfinite(self._period)
                or not 0 <= self._period <= 60):
            raise ValueError("invalid ZipDepth inference parameters")
        if not (root / "zipdepth" / "inference" / "predictor.py").is_file():
            raise RuntimeError(f"ZipDepth checkout missing under {root}")
        if not checkpoint.is_file():
            raise RuntimeError(f"ZipDepth checkpoint missing: {checkpoint}")
        # Checkpoint type and upsampling implementation must agree.
        if checkpoint.name.endswith("_npu.pth") != upsample_npu:
            raise ValueError("NPU checkpoint and upsampling selection differ")
        import torch
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable")
        torch.set_num_threads(threads)
        sys.path.insert(0, str(root))
        from zipdepth.inference.predictor import DepthInference
        self._predictor = DepthInference(
            checkpoint_path=str(checkpoint), variant="base",
            device=device, input_size=input_size, warmup_iters=0,
            upsample_unfold=not upsample_npu,
        )
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._stop = threading.Event()
        self._latest = None
        self._last_stamp = 0
        self._received = 0
        self._processed = 0
        self._replaced = 0
        self._errors = 0
        self._last_duration_ms = 0.0
        self._last_stats = time.monotonic()

        self._publisher = self.create_publisher(
            Image, depth_topic, qos_profile_sensor_data)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._subscription = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data)
        self._worker = threading.Thread(
            target=self._process_frames, name="zipdepth-relative-inference",
            daemon=True)
        self._worker.start()
        self._report("running; relative_only=1 calibrated=0 navigation_authority=0")
        self.get_logger().warning(
            f"ZipDepth RELATIVE depth: {image_topic} -> {depth_topic}; "
            "NOT meters, NOT navigation depth")

    def _report(self, state):
        self._status_pub.publish(String(data=(
            f"{state} received={self._received} processed={self._processed} "
            f"replaced={self._replaced} errors={self._errors} "
            f"last_ms={self._last_duration_ms:.1f}")))

    def _on_image(self, msg):
        with self._lock:
            if self._latest is not None:
                self._replaced += 1
            self._latest = msg
            self._received += 1
        self._frame_ready.set()

    def _process_frames(self):
        while not self._stop.is_set():
            self._frame_ready.wait(0.1)
            self._frame_ready.clear()
            if self._stop.is_set():
                break
            with self._lock:
                msg = self._latest
                self._latest = None
            if msg is None:
                continue
            stamp = (int(msg.header.stamp.sec) * 1_000_000_000
                     + int(msg.header.stamp.nanosec))
            if stamp <= self._last_stamp or stamp <= 0:
                continue
            started = time.monotonic()
            try:
                # ZipDepth's public infer_image API consumes BGR uint8.
                bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                relative = np.asarray(
                    self._predictor.infer_image(np.ascontiguousarray(bgr)),
                    dtype=np.float32)
                if relative.shape != bgr.shape[:2]:
                    raise ValueError("ZipDepth output/image dimensions differ")
                if not np.isfinite(relative).any():
                    raise ValueError("ZipDepth returned no finite predictions")
                relative = relative.copy()
                relative[~np.isfinite(relative)] = np.nan
                output = self._bridge.cv2_to_imgmsg(relative, encoding="32FC1")
                output.header = msg.header
                self._publisher.publish(output)
                self._last_stamp = stamp
                self._processed += 1
            except Exception as exc:
                self._errors += 1
                self.get_logger().error(
                    f"ZipDepth inference: {type(exc).__name__}: {exc}",
                    throttle_duration_sec=5.0)
            self._last_duration_ms = (time.monotonic() - started) * 1000.0
            now = time.monotonic()
            if self._period and now - self._last_stats >= self._period:
                self._report("running; relative_only=1 calibrated=0 navigation_authority=0")
                self._last_stats = now

    def destroy_node(self):
        self._stop.set()
        self._frame_ready.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZipDepthRelativeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

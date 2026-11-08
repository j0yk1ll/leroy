#!/usr/bin/env python3
"""Experimental Metric Video Depth Anything streaming ROS 2 adapter.

Publishes 32FC1 predicted camera-Z on a separate topic for the XRSLAM A1
diagnostic path. This is prototype depth, NOT validated navigation depth.
The source image header is kept unchanged for delayed RGB association.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class MetricVideoDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("metric_video_depth_anything")
        self.declare_parameter("image_topic", "/camera/image_rect")
        self.declare_parameter("depth_topic", "/video_depth_anything/depth")
        self.declare_parameter("status_topic", "/video_depth_anything/status")
        self.declare_parameter("model_root", "/opt/Video-Depth-Anything")
        self.declare_parameter(
            "checkpoint", "checkpoints/metric_video_depth_anything_vits.pth"
        )
        self.declare_parameter("device", "cpu")
        self.declare_parameter("input_size", 280)
        self.declare_parameter("torch_threads", 4)
        self.declare_parameter("stats_period_sec", 5.0)
        self.declare_parameter("max_depth_m", 50.0)

        def param(name):
            return self.get_parameter(name).value

        self.image_topic = str(param("image_topic"))
        self.depth_topic = str(param("depth_topic"))
        self.root = Path(str(param("model_root"))).expanduser().resolve()
        checkpoint = Path(str(param("checkpoint"))).expanduser()
        self.checkpoint = (
            checkpoint if checkpoint.is_absolute() else self.root / checkpoint
        ).resolve()
        self.device_name = str(param("device")).lower()
        self.input_size = int(param("input_size"))
        self.threads = int(param("torch_threads"))
        self.stats_period = float(param("stats_period_sec"))
        self.max_depth = float(param("max_depth_m"))
        if self.input_size < 140 or self.input_size > 700:
            raise ValueError("input_size must be between 140 and 700")
        if self.threads < 1 or self.threads > 64:
            raise ValueError("torch_threads must be between 1 and 64")
        if not 1.0 <= self.max_depth <= 1000.0:
            raise ValueError("max_depth_m out of range")
        if self.device_name == "auto":
            self.device_name = "cuda" if torch.cuda.is_available() else "cpu"
        if self.device_name not in ("cpu", "cuda"):
            raise ValueError("device must be auto, cpu or cuda")
        if self.device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable")
        if not (self.root / "video_depth_anything" / "video_depth_stream.py").is_file():
            raise RuntimeError(
                f"Missing upstream streaming model at {self.root}. "
                "Clone DepthAnything/Video-Depth-Anything to model_root."
            )
        if not self.checkpoint.is_file():
            raise RuntimeError(f"Checkpoint missing: {self.checkpoint}")
        torch.set_num_threads(self.threads)
        self.bridge = CvBridge()

        # Avoid top-level import of upstream modules; the checkout is an
        # external, pinned installation, not copied into Leroy.
        sys.path.insert(0, str(self.root))
        from video_depth_anything.video_depth_stream import VideoDepthAnything

        self.model = VideoDepthAnything(
            encoder="vits", features=64, out_channels=[48, 96, 192, 384]
        )
        try:
            # Only download checkpoints from the official publisher.
            # weights_only=True avoids loading arbitrary Python pickles.
            state = torch.load(
                str(self.checkpoint), map_location="cpu", weights_only=True
            )
            self.model.load_state_dict(state, strict=True)
        except Exception as exc:
            raise RuntimeError(
                "Metric VDA Small weights incompatible with the installed "
                "upstream streaming implementation; verify checkout version "
                f"and checkpoint: {exc}"
            ) from exc
        self.model = self.model.to(self.device_name).eval()

        self._lock = threading.Lock()
        self._latest: Optional[Image] = None
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._received = 0
        self._processed = 0
        self._replaced = 0
        self._errors = 0
        self._last_stamp = -1
        self._last_stats = time.monotonic()
        self._last_ms = 0.0

        qos = sensor_qos()
        self.publisher = self.create_publisher(Image, self.depth_topic, qos)
        self.status_pub = self.create_publisher(
            String, str(param("status_topic")),
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.sub = self.create_subscription(
            Image, self.image_topic, self._on_image, qos
        )
        self.worker = threading.Thread(
            target=self._worker, name="metric-vda-inference", daemon=True
        )
        self.worker.start()
        self._status("running; prototype_only=1 navigation_authority=0")
        self.get_logger().info(
            f"Metric VDA Small {self.device_name} size={self.input_size}: "
            f"{self.image_topic} -> {self.depth_topic} (32FC1 prototype meters)"
        )

    def _status(self, state: str) -> None:
        msg = String()
        msg.data = (
            f"{state} received={self._received} processed={self._processed} "
            f"replaced={self._replaced} errors={self._errors} "
            f"last_ms={self._last_ms:.0f}"
        )
        self.status_pub.publish(msg)

    def _on_image(self, image: Image) -> None:
        with self._lock:
            if self._latest is not None:
                self._replaced += 1
            self._latest = image
            self._received += 1
        self._ready.set()

    def _take_latest(self) -> Optional[Image]:
        with self._lock:
            image = self._latest
            self._latest = None
            return image

    def _worker(self) -> None:
        while not self._stop.is_set():
            self._ready.wait(0.1)
            self._ready.clear()
            if self._stop.is_set():
                break
            msg = self._take_latest()
            if msg is None:
                continue
            stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if stamp <= 0 or stamp <= self._last_stamp:
                continue
            try:
                started = time.monotonic()
                # Upstream streaming inference expects RGB uint8 images.
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                with torch.inference_mode():
                    prediction = self.model.infer_video_depth_one(
                        np.ascontiguousarray(frame),
                        input_size=self.input_size,
                        device=self.device_name,
                        fp32=(self.device_name == "cpu"),
                    )
                depth = np.asarray(prediction, dtype=np.float32)
                if depth.shape != frame.shape[:2]:
                    raise ValueError(
                        f"Unexpected depth dimensions {depth.shape} vs {frame.shape[:2]}"
                    )
                # Reject invalid predictions as NaN; do not invent distances.
                depth = depth.copy()
                depth[~np.isfinite(depth) | (depth <= 0.05) | (depth >= self.max_depth)] = np.nan
                output = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
                output.header = msg.header
                self.publisher.publish(output)
                self._last_stamp = stamp
                self._processed += 1
                self._last_ms = (time.monotonic() - started) * 1000.0
            except Exception as exc:
                self._errors += 1
                self.get_logger().error(
                    f"Metric VDA inference failed: {type(exc).__name__}: {exc}",
                    throttle_duration_sec=5.0,
                )
            now = time.monotonic()
            if self.stats_period > 0 and now - self._last_stats >= self.stats_period:
                self._status("running; prototype_only=1 navigation_authority=0")
                self._last_stats = now

    def destroy_node(self) -> bool:
        self._stop.set()
        self._ready.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MetricVideoDepthNode()
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

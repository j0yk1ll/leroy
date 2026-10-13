#!/usr/bin/env python3
import bisect
import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CamImuSyncTest(Node):
    def __init__(self):
        super().__init__("cam_imu_sync_test")

        self.declare_parameter("camera_topic", "/camera/image_rect")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("report_every", 60)
        self.declare_parameter("imu_buffer_seconds", 5.0)

        self.camera_topic = self.get_parameter("camera_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.report_every = int(self.get_parameter("report_every").value)
        self.imu_buffer_seconds = float(self.get_parameter("imu_buffer_seconds").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.lock = threading.Lock()
        self.imu_times = deque()
        self.dts = []
        self.cam_count = 0
        self.unmatched = 0

        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos)
        self.create_subscription(Image, self.camera_topic, self.on_camera, qos)

        print(f"Camera: {self.camera_topic}")
        print(f"IMU:    {self.imu_topic}")
        print("For each camera frame, computes dt = camera_stamp - nearest_imu_stamp.")
        print("Positive dt means the camera timestamp is later than the nearest IMU timestamp.")
        print("Ctrl-C to stop.\n")

    def on_imu(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        with self.lock:
            self.imu_times.append(t)
            cutoff = t - self.imu_buffer_seconds
            while self.imu_times and self.imu_times[0] < cutoff:
                self.imu_times.popleft()

    def on_camera(self, msg):
        tc = stamp_to_sec(msg.header.stamp)

        with self.lock:
            if not self.imu_times:
                self.unmatched += 1
                return

            times = list(self.imu_times)
            i = bisect.bisect_left(times, tc)

            candidates = []
            if i < len(times):
                candidates.append(times[i])
            if i > 0:
                candidates.append(times[i - 1])

            if not candidates:
                self.unmatched += 1
                return

            ti = min(candidates, key=lambda x: abs(tc - x))
            dt_ms = (tc - ti) * 1000.0

            self.dts.append(dt_ms)
            self.cam_count += 1

            if self.cam_count % self.report_every == 0:
                self.print_report_locked()

    def print_report_locked(self):
        vals = self.dts
        if not vals:
            return

        n = len(vals)
        mean = sum(vals) / n
        var = sum((x - mean) ** 2 for x in vals) / n
        std = math.sqrt(var)

        ordered = sorted(vals)
        p50 = ordered[int(0.50 * (n - 1))]
        p95_abs = sorted(abs(x) for x in vals)[int(0.95 * (n - 1))]

        print(
            f"frames={n:5d}  "
            f"mean={mean:+7.3f} ms  "
            f"std={std:6.3f} ms  "
            f"min={min(vals):+7.3f} ms  "
            f"max={max(vals):+7.3f} ms  "
            f"median={p50:+7.3f} ms  "
            f"p95|dt|={p95_abs:6.3f} ms"
        )

    def final_report(self):
        with self.lock:
            print("\nFinal report")
            self.print_report_locked()
            print(f"unmatched camera frames: {self.unmatched}")


def main():
    rclpy.init()
    node = CamImuSyncTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.final_report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import math
import sys

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuNoiseEstimator(Node):
    def __init__(self):
        super().__init__("imu_noise_estimator")

        self.declare_parameter("topic", "/imu/data_raw")
        self.declare_parameter("duration_s", 180.0)
        self.declare_parameter("warmup_s", 10.0)

        self.topic = str(self.get_parameter("topic").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.warmup_s = float(self.get_parameter("warmup_s").value)

        self.start_ns = self.get_clock().now().nanoseconds
        self.collect_start_ns = None
        self.done = False

        self.stamps_ns = []
        self.gyro = []
        self.accel = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(
            Imu,
            self.topic,
            self.on_imu,
            qos,
        )

        self.get_logger().info(
            f"Listening on {self.topic}; warmup={self.warmup_s:.1f}s, "
            f"collection={self.duration_s:.1f}s. Keep the robot completely stationary."
        )

    def on_imu(self, msg):
        if self.done:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.start_ns) * 1e-9 < self.warmup_s:
            return

        if self.collect_start_ns is None:
            self.collect_start_ns = now_ns
            self.get_logger().info("Warmup complete; collecting samples.")

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        if stamp_ns == 0:
            stamp_ns = now_ns

        self.stamps_ns.append(stamp_ns)
        self.gyro.append(
            [
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ]
        )
        self.accel.append(
            [
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            ]
        )

        if (now_ns - self.collect_start_ns) * 1e-9 >= self.duration_s:
            self.done = True
            self.report()
            rclpy.shutdown()

    @staticmethod
    def vec(v):
        return "[" + ", ".join(f"{x:.9g}" for x in v) + "]"

    @staticmethod
    def flatten_cov(c):
        return [
            c[0, 0],
            c[0, 1],
            c[0, 2],
            c[1, 0],
            c[1, 1],
            c[1, 2],
            c[2, 0],
            c[2, 1],
            c[2, 2],
        ]

    def report(self):
        gyro = np.asarray(self.gyro, dtype=np.float64)
        accel = np.asarray(self.accel, dtype=np.float64)
        stamps = np.asarray(self.stamps_ns, dtype=np.int64)

        if len(gyro) < 10:
            print("Too few samples collected.", file=sys.stderr)
            return

        gyro_mean = gyro.mean(axis=0)
        accel_mean = accel.mean(axis=0)

        gyro_cov = np.cov(gyro, rowvar=False, ddof=1)
        accel_cov = np.cov(accel, rowvar=False, ddof=1)

        gyro_std = np.sqrt(np.diag(gyro_cov))
        accel_std = np.sqrt(np.diag(accel_cov))

        dt = np.diff(stamps) * 1e-9
        dt = dt[dt > 0]
        rate_hz = 1.0 / dt.mean() if len(dt) else float("nan")

        print("\n=== Stationary IMU statistics ===")
        print(f"Samples: {len(gyro)}")
        print(f"Estimated sample rate: {rate_hz:.6f} Hz")

        print("\nAngular velocity [rad/s]")
        print(f"  mean / approximate gyro bias: {self.vec(gyro_mean)}")
        print(f"  stddev:                       {self.vec(gyro_std)}")
        print("  covariance:")
        for row in gyro_cov:
            print("   ", self.vec(row))

        print("\nLinear acceleration [m/s^2]")
        print(f"  mean (includes gravity):      {self.vec(accel_mean)}")
        print(f"  |mean|:                       {np.linalg.norm(accel_mean):.9g}")
        print(f"  stddev:                       {self.vec(accel_std)}")
        print("  covariance:")
        for row in accel_cov:
            print("   ", self.vec(row))

        print("\nROS covariance arrays")
        print("angular_velocity_covariance:")
        print("  [" + ", ".join(f"{x:.12g}" for x in self.flatten_cov(gyro_cov)) + "]")
        print("linear_acceleration_covariance:")
        print("  [" + ", ".join(f"{x:.12g}" for x in self.flatten_cov(accel_cov)) + "]")

        if math.isfinite(rate_hz) and rate_hz > 0:
            gyro_nd = gyro_std / math.sqrt(rate_hz)
            accel_nd = accel_std / math.sqrt(rate_hz)
            print("\nRough white-noise-density estimate")
            print("  first-pass only; use Allan variance for VIO calibration")
            print(f"  gyro  [rad/s/sqrt(Hz)]:  {self.vec(gyro_nd)}")
            print(f"  accel [m/s^2/sqrt(Hz)]: {self.vec(accel_nd)}")

        print("\nNotes:")
        print("  - Gyro stationary mean is a useful bias estimate.")
        print(
            "  - Accelerometer mean includes gravity, so it is not accelerometer bias."
        )
        print(
            "  - Sample covariance is suitable as a first-pass sensor_msgs/Imu covariance."
        )
        print(
            "  - VIO noise-density/random-walk parameters should later come from Allan deviation."
        )
        print("=================================")


def main(args=None):
    rclpy.init(args=args)
    node = ImuNoiseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.done and len(node.gyro) >= 10:
            node.report()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()

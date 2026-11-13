#!/usr/bin/env python3
"""Diagnostic-only ROS adapter for relative-depth discontinuities.

No occupancy grid, free-space inference, navigation or motor authority.
"""
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from leroy_perception.relative_depth_structure import (
    detect_relative_boundaries,
)


class RelativeDepthBoundaryNode(Node):

    def __init__(self):
        super().__init__("relative_depth_boundary_diagnostic")

        self.declare_parameter(
            "input_topic", "/relative_depth/zipdepth"
        )
        self.declare_parameter(
            "output_topic", "/relative_depth/boundary_diagnostic"
        )
        self.declare_parameter("contrast_threshold", 0.15)
        self.declare_parameter("min_valid_fraction", 0.50)

        p = lambda name: self.get_parameter(name).value

        threshold = float(p("contrast_threshold"))
        valid_fraction = float(p("min_valid_fraction"))

        if (
            not math.isfinite(threshold)
            or not 0 < threshold < 1
            or not math.isfinite(valid_fraction)
            or not 0 < valid_fraction <= 1
        ):
            raise ValueError("Invalid relative-depth diagnostic parameters")

        self.threshold = threshold
        self.valid_fraction = valid_fraction
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image, str(p("output_topic")), qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image,
            str(p("input_topic")),
            self.on_depth,
            qos_profile_sensor_data,
        )

        self.get_logger().warning(
            "RELATIVE DEPTH DIAGNOSTIC ONLY: "
            "possible boundaries, no free-space or navigation authority"
        )

    def on_depth(self, msg):
        try:
            if msg.encoding != "32FC1":
                raise ValueError("Expected 32FC1 relative-depth image")

            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="32FC1"
            )

            mask = detect_relative_boundaries(
                image,
                contrast_threshold=self.threshold,
                min_valid_fraction=self.valid_fraction,
            )

            output = self.bridge.cv2_to_imgmsg(
                np.ascontiguousarray(mask), encoding="mono8"
            )
            output.header = msg.header

            self.publisher.publish(output)

        except (ValueError, TypeError, RuntimeError) as exc:
            self.get_logger().warning(
                f"Relative-depth diagnostic rejected frame: {exc}",
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RelativeDepthBoundaryNode()
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

"""Realtime optional camera overlay image; no actuation or pose authority."""
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image

from leroy_navigation.feature_overlay import draw_corners


class FeatureOverlayNode(Node):
    def __init__(self):
        super().__init__("leroy_feature_overlay")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("overlay_topic", "/leroy/visualization/features")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, str(self.get_parameter("overlay_topic").value), 5)
        self.subscription = self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self.on_image, 5)

    def on_image(self, msg):
        if self.publisher.get_subscription_count() == 0:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            overlay = draw_corners(image)
            out = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out.header = msg.header
            self.publisher.publish(out)
        except (ValueError, CvBridgeError) as exc:
            self.get_logger().warning("Feature frame skipped: %s" % str(exc),
                                      throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = FeatureOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

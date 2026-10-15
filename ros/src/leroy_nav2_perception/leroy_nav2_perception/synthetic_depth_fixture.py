"""Explicitly synthetic 4x4 depth camera fixture, for offline Podman ONLY."""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class SyntheticDepthFixture(Node):
    def __init__(self):
        super().__init__('synthetic_depth_fixture')
        self._depth_pub = self.create_publisher(
            Image, '/depthart/depth', qos_profile_sensor_data)
        self._info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self._tf = StaticTransformBroadcaster(self)
        # SYNTHETIC camera at z=0.2 above SYNTHETIC base_link-floor origin.
        # Exactly maps optical (+x right,+y down,+z forward) to
        # base (+x forward,+y left,+z up). NOT Leroy's real mounting geometry.
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'sim_camera_optical_frame'
        tf.transform.translation.z = 0.2
        tf.transform.rotation.x = 0.5
        tf.transform.rotation.y = -0.5
        tf.transform.rotation.z = 0.5
        tf.transform.rotation.w = -0.5
        self._tf.sendTransform(tf)
        self.create_timer(0.5, self._publish)
        self.get_logger().warning(
            'SYNTHETIC ONLY; 4x4 fake depth and TF, not sensor or calibration')

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'sim_camera_optical_frame'
        info.width = 4
        info.height = 4
        info.p = [10.0, 0.0, 1.5, 0.0,
                  0.0, 10.0, 1.5, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.k = [10.0, 0.0, 1.5, 0.0, 10.0, 1.5, 0.0, 0.0, 1.0]
        img = Image()
        img.header = info.header
        img.width = 4
        img.height = 4
        img.encoding = '32FC1'
        img.is_bigendian = False
        img.step = 16
        img.data = np.ones((4, 4), dtype='<f4').tobytes()
        self._info_pub.publish(info)
        self._depth_pub.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticDepthFixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

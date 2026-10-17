"""M21D passive ROS camera metadata adapter; NOT a runnable hardware launcher.

Never opens serial/video, never publishes ROS topics, and never creates a
motor-capable node. The future reviewed source integration must explicitly
call record_from_m18_source after a successful validated M18 IMU publication.
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from m20_combined_observer import check_isolated_graph
from m21d_metadata_capture import MetadataCapture


EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1", "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}


def record_from_m18_source(capture, fields, mapped_ros_ns, receipt_mono_ns):
    """Pass ONLY actual parsed MCU fields and M18 mapped-publish timestamp.

    This callable must be wired at the M18 source after successful publish,
    not reconstructed from sensor_msgs/Imu and not inferred from ROS receipt.
    """
    if fields["cal"] != 1:
        raise ValueError("cannot record IMU metadata for unpublished packet")
    capture.imu_source(
        mcu_seq=fields["seq"], mcu_t_us=fields["t_us"],
        stamp_ns=mapped_ros_ns, receipt_mono_ns=receipt_mono_ns,
        gyro_xyz_rad_s=[fields["gr_mrad_s"] / 1000.0,
                        fields["gp_mrad_s"] / 1000.0,
                        fields["gy_mrad_s"] / 1000.0])


class PassiveCameraMetadataNode(Node):
    """Read metadata of existing camera ROS messages, never retain pixel data."""
    def __init__(self, capture):
        for key, value in EXPECTED_ENV.items():
            if os.environ.get(key) != value:
                raise RuntimeError("M21D requires M20 isolated sensor-only environment")
        if not isinstance(capture, MetadataCapture):
            raise TypeError("M21D requires validated metadata capture")
        super().__init__("m21d_camera_metadata_observer", enable_rosout=False,
                         start_parameter_services=False)
        self.capture = capture
        self.frame_index = 0
        self._subscriptions = [
            self.create_subscription(Image, "/camera/image_rect",
                                     self._image, qos_profile_sensor_data),
            self.create_subscription(CameraInfo, "/camera/camera_info",
                                     self._info, qos_profile_sensor_data)]
        if [sub.topic_name for sub in self._subscriptions] != [
                "/camera/image_rect", "/camera/camera_info"]:
            raise RuntimeError("M21D refuses camera topic remapping")

    def _verify_graph(self, topic, node, namespace, msgtype):
        check_isolated_graph(self)
        endpoints = self.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1:
            raise RuntimeError("M21D requires one known camera publisher")
        ep = endpoints[0]
        if (ep.node_name, ep.node_namespace, ep.topic_type) != (
                node, namespace, msgtype):
            raise RuntimeError("M21D wrong camera publisher provenance")

    def _image(self, msg):
        receipt_mono_ns = time.monotonic_ns()
        self._verify_graph("/camera/image_rect", "rectify_node", "/camera",
                           "sensor_msgs/msg/Image")
        self.capture.image(
            frame_index=self.frame_index,
            stamp_ns=MetadataCapture.stamp_ns(msg.header.stamp.sec,
                                               msg.header.stamp.nanosec),
            receipt_mono_ns=receipt_mono_ns,
            frame_id=msg.header.frame_id, width=msg.width, height=msg.height)
        self.frame_index += 1

    def _info(self, msg):
        receipt_mono_ns = time.monotonic_ns()
        self._verify_graph("/camera/camera_info", "usb_cam", "/camera",
                           "sensor_msgs/msg/CameraInfo")
        self.capture.camera_info(
            stamp_ns=MetadataCapture.stamp_ns(msg.header.stamp.sec,
                                               msg.header.stamp.nanosec),
            receipt_mono_ns=receipt_mono_ns,
            frame_id=msg.header.frame_id, width=msg.width, height=msg.height)

# Intentionally NO main(), rclpy.init(), serial.Serial(), image acquisition,
# launch file or executable entrypoint. Importing this module cannot start
# physical sensors; M21D is offline schema/writer and ROS-adapter readiness.

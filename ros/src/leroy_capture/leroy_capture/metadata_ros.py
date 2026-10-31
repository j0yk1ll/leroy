"""Stage 4C ROS message wrapper for the M18 source metadata hook.

Purely a COMPONENT, not an executable or enabled runtime integration.
Importing this module never initializes ROS or a serial port. Future hardware
integration requires an independently reviewed, explicitly authorized
launcher and image; the accepted Stage 4B runner remains unchanged.
"""
from leroy_capture.contract import CaptureError, METADATA_TOPIC
from leroy_capture.source_metadata import SourceMetadataTap


class SourceMetadataRosAdapter:
    """Create only one source-side std_msgs/String metadata publisher.

    The provided node MUST be the same existing /m18_imu_source instance
    that is actually publishing /imu/data_raw. Caller supplies this after
    constructing the accepted M18 node; no additional source or serial
    endpoint is instantiated here.
    """

    def __init__(self, source_node, *, message_type=None, qos=None,
                 monotonic_ns=None):
        if (not hasattr(source_node, "get_name") or
                not hasattr(source_node, "get_namespace") or
                not hasattr(source_node, "create_publisher") or
                source_node.get_name() != "m18_imu_source" or
                source_node.get_namespace() != "/"):
            raise CaptureError("source_node_identity_not_m18")
        # Lazy ROS imports preserve pure-Python, no-device regression tests.
        if message_type is None:
            from std_msgs.msg import String
            message_type = String
        if qos is None:
            from rclpy.qos import qos_profile_sensor_data
            qos = qos_profile_sensor_data
        if not callable(message_type):
            raise CaptureError("metadata_string_message_type_missing")
        publisher = source_node.create_publisher(
            message_type, METADATA_TOPIC, qos)
        if getattr(publisher, "topic_name", None) != METADATA_TOPIC:
            raise CaptureError("metadata_topic_remapped")
        if not callable(getattr(publisher, "publish", None)):
            raise CaptureError("metadata_publisher_invalid")
        self._publisher = publisher
        self._message_type = message_type

        def emit(serialized):
            msg = self._message_type()
            msg.data = serialized
            self._publisher.publish(msg)

        kwargs = {"monotonic_ns": monotonic_ns} if monotonic_ns is not None else {}
        self.tap = SourceMetadataTap(emit, **kwargs)

    def on_imu_published(self, fields, stamp_ns, mapper_steady_ns):
        """Forward the actual M18 *after successful IMU publish* callback."""
        return self.tap.on_imu_published(fields, stamp_ns, mapper_steady_ns)

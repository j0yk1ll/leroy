"""Integration smoke test for Leroy's ROS 2 Jazzy rosbag2_py bindings.

Run *inside* the Leroy ROS container after sourcing ROS Jazzy and /leroy_ws:
    python3 -m unittest discover -s /tmp/leroy_utils/imu -p test_rosbag_writer.py -v

This test does not start ROS nodes, subscribe to the camera, or command motors.
It creates a disposable local SQLite bag with one synthetic message per topic.
"""
import gc
from pathlib import Path
import tempfile
import unittest

try:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, CompressedImage, Imu
except ImportError:
    rosbag2_py = None


@unittest.skipIf(rosbag2_py is None, "ROS 2 Jazzy Python packages unavailable")
class TestJazzyRosbagWriter(unittest.TestCase):
    def test_create_topics_write_and_read(self):
        topics = [
            ("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage", CompressedImage()),
            ("/camera/camera_info", "sensor_msgs/msg/CameraInfo", CameraInfo()),
            ("/imu/data_raw", "sensor_msgs/msg/Imu", Imu()),
        ]
        with tempfile.TemporaryDirectory(prefix="leroy-rosbag-smoke-") as tempdir:
            uri = str(Path(tempdir) / "bag")
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"),
            )
            for name, message_type, _ in topics:
                # ROS 2 Jazzy's Python TopicMetadata constructor requires id.
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=0,
                    name=name,
                    type=message_type,
                    serialization_format="cdr",
                ))
            for name, _, message in topics:
                writer.write(name, serialize_message(message), 1_791_992_400_000_000_000)
            if hasattr(writer, "close"):
                writer.close()
            del writer
            gc.collect()

            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"),
            )
            registered = {meta.name for meta in reader.get_all_topics_and_types()}
            self.assertEqual(registered, {name for name, _, _ in topics})
            observed = []
            while reader.has_next():
                name, _, _ = reader.read_next()
                observed.append(name)
            self.assertCountEqual(observed, [name for name, _, _ in topics])
            del reader


if __name__ == "__main__":
    unittest.main()

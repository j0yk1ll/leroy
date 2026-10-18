"""M23B: deterministic synthetic rectified monocular ROS bag, hardware-free.

No ROS nodes, sensors, MCU, serial, motors or XRSLAM are opened here.
Only the Jazzy rosbag2 writer and synthetic ROS message constructors run.
Synthetic pixels and identity TF are NEVER measurements or calibration.
"""
import gc
from pathlib import Path

FRAME_ID = "camera_optical_frame"
WIDTH, HEIGHT = 640, 480
FRAME_COUNT = 16
PERIOD_NS = 125_000_000
START_NS = 1_800_000_000_000_000_000


def _header(message, stamp_ns):
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = FRAME_ID
    return message


def synthetic_pixels(frame_index):
    """High-contrast evolving texture, wholly fabricated."""
    if not 0 <= frame_index < FRAME_COUNT:
        raise ValueError("fixture index out of bounds")
    # Slight translation to prevent every frame being byte-identical.
    line = bytes(
        (255 if (((x + frame_index * 2) // 16 + (y // 16)) & 1)
         else 0)
        for y in range(16) for x in range(WIDTH))
    return line * (HEIGHT // 16)


def synthetic_records():
    from sensor_msgs.msg import CameraInfo, Image
    for index in range(FRAME_COUNT):
        stamp = START_NS + index * PERIOD_NS
        info = _header(CameraInfo(), stamp)
        info.width = WIDTH
        info.height = HEIGHT
        info.p = [400.0, 0.0, WIDTH / 2.0, 0.0,
                  0.0, 400.0, HEIGHT / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        im = _header(Image(), stamp)
        im.width, im.height = WIDTH, HEIGHT
        im.encoding = "mono8"
        im.is_bigendian = 0
        im.step = WIDTH
        im.data = synthetic_pixels(index)
        yield stamp, "/camera/camera_info", info
        yield stamp + 1, "/camera/image_rect", im


def write_synthetic_bag(directory):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    if not isinstance(directory, Path) or directory.exists():
        raise ValueError("bag must be a new Path")
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(
        uri=str(directory), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic, ros_type in (
            ("/camera/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("/camera/image_rect", "sensor_msgs/msg/Image")):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=topic, type=ros_type, serialization_format="cdr"))
    for at, topic, message in synthetic_records():
        writer.write(topic, serialize_message(message), at)
    if hasattr(writer, "close"):
        writer.close()
    del writer
    gc.collect()
    if not (directory / "metadata.yaml").is_file():
        raise AssertionError("no bag metadata produced")
    return directory

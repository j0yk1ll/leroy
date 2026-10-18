"""M23C: synthetic camera+IMU CDR fixture for NO-DEVICE XRSLAM VIO.

All imagery, inertial values and timing are generated test inputs, not
measured on Leroy. No hardware, sensors, serial, ROS nodes or motor code.
A deliberate IMU dropout is INJECTED after the valid bag has completed;
it is NOT recorded as healthy source data.
"""
import gc
from pathlib import Path

from m23b_synthetic_monocular_bag import (
    START_NS, FRAME_COUNT, PERIOD_NS, synthetic_records as camera_records,
)

IMU_PERIOD_NS = 20_000_000  # Synthetic 50 Hz
IMU_START_NS = START_NS - 40_000_000
IMU_COUNT = 104
IMU_FRAME = "imu_link"
IMU_TOPIC = "/imu/data_raw"
LAST_IMU_NS = IMU_START_NS + (IMU_COUNT - 1) * IMU_PERIOD_NS
DROPOUT_NS = 500_000_000


def imu_stamps():
    return tuple(IMU_START_NS + i * IMU_PERIOD_NS
                 for i in range(IMU_COUNT))


def schedule_metadata():
    """Pure-Python expected source counts and monotone CDR receipt timeline."""
    cam = tuple((at, topic) for at, topic, _msg in camera_records_import_free())
    imu = tuple((at, IMU_TOPIC) for at in imu_stamps())
    return tuple(sorted(cam + imu))


def camera_records_import_free():
    # Pure timing contract, matching M23B's camera generator.
    for i in range(FRAME_COUNT):
        at = START_NS + i * PERIOD_NS
        yield at, "/camera/camera_info", None
        yield at + 1, "/camera/image_rect", None


def synthetic_vio_records():
    from sensor_msgs.msg import Imu
    events = list(camera_records())
    for stamp in imu_stamps():
        m = Imu()
        m.header.stamp.sec = stamp // 1_000_000_000
        m.header.stamp.nanosec = stamp % 1_000_000_000
        m.header.frame_id = IMU_FRAME
        # Fixed finite fabricated gyro/accel values; no measured noise.
        m.angular_velocity.z = 0.01
        m.linear_acceleration.z = 9.81
        events.append((stamp, IMU_TOPIC, m))
    yield from sorted(events, key=lambda item: item[0])


def write_synthetic_vio_bag(directory):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    if not isinstance(directory, Path) or directory.exists():
        raise ValueError("target must be a new bag directory")
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(directory), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    for name, ros_type in (
        ("/camera/camera_info", "sensor_msgs/msg/CameraInfo"),
        ("/camera/image_rect", "sensor_msgs/msg/Image"),
        (IMU_TOPIC, "sensor_msgs/msg/Imu"),
    ):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=name, type=ros_type, serialization_format="cdr"))
    for receipt, topic, message in synthetic_vio_records():
        writer.write(topic, serialize_message(message), receipt)
    if hasattr(writer, "close"):
        writer.close()
    del writer
    gc.collect()
    if not (directory / "metadata.yaml").is_file():
        raise AssertionError("synthetic rosbag metadata absent")
    return directory

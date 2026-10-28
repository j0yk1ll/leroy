"""Stage 4D shared, fail-closed Stage 4C ROS message ingress contract.

This module has NO ROS import, hardware access, filesystem operation, thread,
publisher or motor API. The Stage 4C recorder and offline fake sensor tests
use the same message validation and serialized-write call path.

The optional injected monotonic clock is ONLY receipt context. The physical
camera exposure and MCU-to-camera absolute phase remain unknown.
"""
import math
import time

from live_stage4c_capture_contract import (
    CaptureContract, CaptureError, METADATA_TOPIC, TOPICS, source_metadata,
)


class CaptureIngress:
    """Validate one received ROS-like message and atomically account writes.

    serialize(msg) must return CDR bytes. write(topic, raw, stamp) is an
    injected bag writer. No files or DDS connections are created here.
    Once any callback fails, further accepts are denied until a new
    instance is created. The owning runner must abort and close the bag.
    """

    def __init__(self, *, contract=None, serialize, write,
                 monotonic_ns=time.monotonic_ns):
        if not callable(serialize) or not callable(write) or not callable(monotonic_ns):
            raise CaptureError("ingress_callbacks_required")
        self.contract = contract if contract is not None else CaptureContract()
        self._serialize = serialize
        self._write = write
        self._clock = monotonic_ns
        self.terminal_fault = None

    def accept(self, topic, msg, *, receipt_ns=None):
        if self.terminal_fault is not None:
            raise CaptureError("ingress_terminal_fault_latched")
        try:
            if topic not in TOPICS:
                raise CaptureError("unexpected_topic")
            received = self._clock() if receipt_ns is None else receipt_ns
            metadata = None
            if topic == METADATA_TOPIC:
                metadata = source_metadata(msg.data)
                stamp = metadata["imu_stamp_ns"]
                frame = None
            else:
                stamp = (int(msg.header.stamp.sec) * 1_000_000_000
                         + int(msg.header.stamp.nanosec))
                frame = msg.header.frame_id

            if topic == "/camera/image_rect":
                width = msg.width
                height = msg.height
                step = msg.step
                if (type(width) is not int or type(height) is not int
                        or type(step) is not int or width <= 0 or height <= 0
                        or step < width or not isinstance(msg.encoding, str)
                        or not msg.encoding or len(msg.data) != height * step):
                    raise CaptureError("invalid_image_geometry_or_payload")
            elif topic == "/camera/camera_info":
                if (msg.width <= 0 or msg.height <= 0
                        or len(msg.k) != 9 or
                        not all(math.isfinite(v) for v in (*msg.k, *msg.d))
                        or msg.k[0] <= 0 or msg.k[4] <= 0):
                    raise CaptureError("invalid_camera_intrinsics")
            elif topic == "/imu/data_raw":
                values = (
                    msg.angular_velocity.x, msg.angular_velocity.y,
                    msg.angular_velocity.z, msg.linear_acceleration.x,
                    msg.linear_acceleration.y, msg.linear_acceleration.z,
                )
                if not all(math.isfinite(v) for v in values):
                    raise CaptureError("nonfinite_imu_measurement")

            raw = self._serialize(msg)
            self.contract.accept(
                topic, stamp, frame, raw, received, self._write,
                metadata=metadata)
            return stamp
        except Exception as exc:
            self.terminal_fault = type(exc).__name__ + ":" + str(exc)
            raise

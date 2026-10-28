"""Stage 4C pure-Python bounded, fail-closed passive calibration capture contract.

No ROS, serial, camera, motor or filesystem I/O imports. The contract does
not measure exposure phase or certify physical calibration.
"""
import json
import math

TOPICS = {
    "/camera/image_rect": ("sensor_msgs/msg/Image", "camera_optical_frame"),
    "/camera/camera_info": ("sensor_msgs/msg/CameraInfo", "camera_optical_frame"),
    "/imu/data_raw": ("sensor_msgs/msg/Imu", "imu_link"),
    "/stage4c/imu_source_meta": ("std_msgs/msg/String", None),
}
METADATA_TOPIC = "/stage4c/imu_source_meta"
MAX_DURATION_S = 75
MAX_RECORDS = 20000
MAX_SERIALIZED_BYTES = 900_000_000
MAX_MESSAGE_BYTES = 4_000_000
META_FIELDS = {
    "schema", "publisher_node", "seq", "t_us", "imu_stamp_ns",
    "publish_monotonic_ns", "cal",
}


class CaptureError(ValueError):
    pass


def _integer(value, label, minimum=0, maximum=None):
    if type(value) is not int or value < minimum or (
        maximum is not None and value > maximum
    ):
        raise CaptureError("invalid_" + label)
    return value


def source_metadata(data):
    """Validate source-originated MCU fields, not inferred ROS header data."""
    if type(data) is str:
        if len(data) > 512:
            raise CaptureError("metadata_too_long")
        try:
            data = json.loads(data, object_pairs_hook=_no_duplicate_keys)
        except (ValueError, TypeError) as exc:
            raise CaptureError("invalid_metadata_json") from exc
    if not isinstance(data, dict) or set(data) != META_FIELDS:
        raise CaptureError("invalid_metadata_shape")
    if (data["schema"] != "leroy_stage4c_mcu_v1"
            or data["publisher_node"] != "/m18_imu_source"
            or type(data["cal"]) is not int or data["cal"] != 1):
        raise CaptureError("invalid_metadata_provenance")
    _integer(data["seq"], "sequence", maximum=4_294_967_295)
    _integer(data["t_us"], "mcu_ticks", maximum=4_294_967_295)
    _integer(data["imu_stamp_ns"], "mcu_mapped_stamp", minimum=1)
    _integer(data["publish_monotonic_ns"], "source_monotonic", minimum=1)
    return data


def _no_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise CaptureError("duplicate_metadata_key")
        result[key] = value
    return result


class CaptureContract:
    def __init__(self, *, duration_s=MAX_DURATION_S,
                 max_records=MAX_RECORDS,
                 max_bytes=MAX_SERIALIZED_BYTES):
        if type(duration_s) not in (int, float) or not math.isfinite(duration_s) or not 1 <= duration_s <= MAX_DURATION_S:
            raise CaptureError("invalid_duration")
        _integer(max_records, "record_limit", 1, MAX_RECORDS)
        _integer(max_bytes, "byte_limit", 1, MAX_SERIALIZED_BYTES)
        self.duration_s = duration_s
        self.max_records = max_records
        self.max_bytes = max_bytes
        self.counts = dict.fromkeys(TOPICS, 0)
        self.last_stamp = dict.fromkeys(TOPICS, 0)
        self.imu_stamps = set()
        self.meta_stamps = set()
        self.last_seq = None
        self.bytes_written = 0
        self.first_receipt_ns = None
        self.last_receipt_ns = None

    def accept(self, topic, stamp_ns, frame_id, payload, receipt_ns, writer, *,
               metadata=None):
        """Validate then write, updating accounting only after writer succeeds."""
        if topic not in TOPICS:
            raise CaptureError("unexpected_topic")
        _integer(stamp_ns, "stamp", 1)
        _integer(receipt_ns, "receipt_monotonic", 1)
        if stamp_ns <= self.last_stamp[topic]:
            raise CaptureError("nonmonotonic_stamp")
        if self.last_receipt_ns is not None and receipt_ns < self.last_receipt_ns:
            raise CaptureError("nonmonotonic_receipt")
        if self.first_receipt_ns is not None and (
            receipt_ns - self.first_receipt_ns > self.duration_s * 1_000_000_000
        ):
            raise CaptureError("duration_limit")
        if frame_id != TOPICS[topic][1]:
            raise CaptureError("incorrect_frame")
        if not isinstance(payload, bytes) or not 0 < len(payload) <= MAX_MESSAGE_BYTES:
            raise CaptureError("invalid_serialized_payload")
        if sum(self.counts.values()) >= self.max_records:
            raise CaptureError("record_limit")
        if self.bytes_written + len(payload) > self.max_bytes:
            raise CaptureError("serialized_byte_limit")
        meta = None
        if topic == METADATA_TOPIC:
            meta = source_metadata(metadata)
            if stamp_ns != meta["imu_stamp_ns"]:
                raise CaptureError("metadata_stamp_mismatch")
            if meta["publish_monotonic_ns"] > receipt_ns:
                raise CaptureError("metadata_receipt_before_publication")
            if self.last_seq is not None and (
                meta["seq"] - self.last_seq
            ) % (1 << 32) != 1:
                raise CaptureError("mcu_sequence_gap")
        elif metadata is not None:
            raise CaptureError("unexpected_metadata")
        if topic == "/imu/data_raw" and stamp_ns in self.imu_stamps:
            raise CaptureError("duplicate_imu_stamp")
        if topic == METADATA_TOPIC and stamp_ns in self.meta_stamps:
            raise CaptureError("duplicate_meta_stamp")
        if not callable(writer):
            raise CaptureError("missing_writer")
        writer(topic, payload, stamp_ns)
        self.counts[topic] += 1
        self.last_stamp[topic] = stamp_ns
        self.bytes_written += len(payload)
        self.last_receipt_ns = receipt_ns
        if self.first_receipt_ns is None:
            self.first_receipt_ns = receipt_ns
        if topic == "/imu/data_raw":
            self.imu_stamps.add(stamp_ns)
        if meta is not None:
            self.last_seq = meta["seq"]
            self.meta_stamps.add(stamp_ns)

    def report(self):
        image_count = self.counts["/camera/image_rect"]
        info_count = self.counts["/camera/camera_info"]
        imu_count = self.counts["/imu/data_raw"]
        meta_count = self.counts[METADATA_TOPIC]
        matched = len(self.imu_stamps & self.meta_stamps)
        complete = (image_count >= 100 and info_count >= 100 and
                    imu_count >= 500 and meta_count == imu_count and
                    matched == imu_count)
        return {
            "schema": "leroy_stage4c_capture_contract_v1",
            "counts": dict(self.counts),
            "total_serialized_bytes": self.bytes_written,
            "matched_imu_metadata_stamps": matched,
            "unmatched_imu_stamps": imu_count - matched,
            "unmatched_metadata_stamps": meta_count - matched,
            "source_metadata_provenance_verified": complete,
            "readiness": "COMPLETE_INPUTS_FOR_OFFLINE_ANALYSIS" if complete
                         else "BLOCKED_INCOMPLETE_CAPTURE",
            "camera_exposure_semantics_validated": False,
            "physical_camera_imu_offset_validated": False,
            "imu_noise_validated": False,
            "spatial_extrinsics_validated": False,
            "xrslam_vio_authorized": False,
            "metric_odometry_authorized": False,
            "motor_commands_sent": 0,
        }

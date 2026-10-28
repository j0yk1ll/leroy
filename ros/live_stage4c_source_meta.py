"""Stage 4C M18 post-publication source metadata adapter; NO device access.

This module does not create a ROS node or publisher and is deliberately NOT
wired into the accepted Stage 4B serial runtime. It is callable only from
M18's existing on_imu_published(fields, stamp_ns, steady_ns) hook AFTER
the IMU publish() returns. Production integration requires another review.

The monotonic tick here is taken AFTER the post-publish callback starts,
not the M18 clock-mapper observation passed as _mapper_steady_ns.
"""
import json
import time

from live_stage4c_capture_contract import CaptureError, source_metadata

UINT32 = 1 << 32
MAX_TICK_DELTA_US = 1_000_000


class SourceMetadataTap:
    """Fail-closed source-side metadata builder and injected sender."""

    def __init__(self, send, *, monotonic_ns=time.monotonic_ns):
        if not callable(send) or not callable(monotonic_ns):
            raise CaptureError("metadata_tap_requires_callbacks")
        self._send = send
        self._clock = monotonic_ns
        self._last_seq = None
        self._last_t_us = None
        self._last_stamp_ns = 0
        self._last_publish_ns = 0
        self.count = 0
        self.terminal_fault = None

    def on_imu_published(self, fields, stamp_ns, _mapper_steady_ns):
        """M18 metadata only; never reads serial or issues a second IMU publish."""
        if self.terminal_fault is not None:
            raise CaptureError("metadata_tap_fault_latched")
        try:
            if not isinstance(fields, dict) or any(
                name not in fields for name in ("seq", "t_us", "cal")
            ):
                raise CaptureError("missing_original_mcu_fields")
            publish_ns = self._clock()
            data = source_metadata({
                "schema": "leroy_stage4c_mcu_v1",
                "publisher_node": "/m18_imu_source",
                "seq": fields["seq"],
                "t_us": fields["t_us"],
                "imu_stamp_ns": stamp_ns,
                "publish_monotonic_ns": publish_ns,
                "cal": fields["cal"],
            })
            if (data["imu_stamp_ns"] <= self._last_stamp_ns or
                    data["publish_monotonic_ns"] <= self._last_publish_ns):
                raise CaptureError("nonmonotonic_source_timestamp")
            if self._last_seq is not None:
                if (data["seq"] - self._last_seq) % UINT32 != 1:
                    raise CaptureError("source_sequence_gap")
                delta_us = (data["t_us"] - self._last_t_us) % UINT32
                if not 1 <= delta_us <= MAX_TICK_DELTA_US:
                    raise CaptureError("source_mcu_tick_discontinuity")
            serialized = json.dumps(
                data, sort_keys=True, separators=(",", ":"), allow_nan=False)
            # Injected sender. A future wrapper can publish std_msgs/String,
            # but this pure module does not import ROS or expose a live entry.
            self._send(serialized)
            # Only commit counts/provenance AFTER sender returns successfully.
            self._last_seq = data["seq"]
            self._last_t_us = data["t_us"]
            self._last_stamp_ns = data["imu_stamp_ns"]
            self._last_publish_ns = data["publish_monotonic_ns"]
            self.count += 1
            return serialized
        except Exception as exc:
            self.terminal_fault = type(exc).__name__ + ":" + str(exc)
            raise

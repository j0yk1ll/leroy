"""Bounded, device-free exact-stamp match of actual M18 IMU publish/receive callbacks.

Matching records can characterize the source-to-observer scheduling path in
one ROS process. They cannot by themselves distinguish Fast DDS losses from
executor backlog, nor estimate the physical camera-IMU time offset.
"""
import math

MAX_SAMPLES = 10000


def _integer(value, label, min_value=1):
    if type(value) is not int or value < min_value:
        raise ValueError("invalid_" + label)


def _percentile(values, fraction):
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lo = int(index)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


class DeliveryTrace:
    def __init__(self, max_samples=MAX_SAMPLES):
        _integer(max_samples, "max_samples")
        if max_samples > MAX_SAMPLES:
            raise ValueError("too_many_samples")
        self.max_samples = max_samples
        self._source = {}
        self._subscriber = {}
        self._source_out_of_order = False
        self._subscriber_out_of_order = False
        self._last_source_stamp = 0
        self._last_subscriber_stamp = 0

    def source(self, stamp_ns, callback_mono_ns):
        """Call only from M18's on_imu_published hook, AFTER publish()."""
        _integer(stamp_ns, "source_stamp")
        _integer(callback_mono_ns, "source_mono")
        if stamp_ns in self._source or len(self._source) >= self.max_samples:
            raise ValueError("source_duplicate_or_capacity")
        if stamp_ns <= self._last_source_stamp:
            self._source_out_of_order = True
            raise ValueError("source_timestamp_regressed")
        self._source[stamp_ns] = callback_mono_ns
        self._last_source_stamp = stamp_ns

    def subscriber(self, stamp_ns, callback_mono_ns):
        """Call when the passive ROS Imu subscriber actually processes a sample."""
        _integer(stamp_ns, "subscriber_stamp")
        _integer(callback_mono_ns, "subscriber_mono")
        if stamp_ns in self._subscriber or len(self._subscriber) >= self.max_samples:
            raise ValueError("subscriber_duplicate_or_capacity")
        if stamp_ns <= self._last_subscriber_stamp:
            self._subscriber_out_of_order = True
            raise ValueError("subscriber_timestamp_regressed")
        self._subscriber[stamp_ns] = callback_mono_ns
        self._last_subscriber_stamp = stamp_ns

    def report(self):
        shared = self._source.keys() & self._subscriber.keys()
        delays_ms = []
        negative_latency = 0
        for stamp in shared:
            dt_ns = self._subscriber[stamp] - self._source[stamp]
            if dt_ns < 0:
                negative_latency += 1
                continue
            delays_ms.append(dt_ns / 1e6)
        # Do not fabricate a measured latency when no two records match.
        valid = bool(delays_ms and not negative_latency
                     and not self._source_out_of_order
                     and not self._subscriber_out_of_order)
        return {
            "trace_schema": "leroy_stage4b_imu_stamp_v1",
            "source_after_publish_callbacks": len(self._source),
            "subscriber_receive_callbacks": len(self._subscriber),
            "matched_exact_header_stamps": len(shared),
            "source_unmatched_stamps": len(self._source) - len(shared),
            "subscriber_unmatched_stamps": len(self._subscriber) - len(shared),
            "match_fraction_of_source": (
                round(len(shared) / len(self._source), 6)
                if self._source else None),
            "negative_host_monotonic_delays": negative_latency,
            "source_out_of_order": self._source_out_of_order,
            "subscriber_out_of_order": self._subscriber_out_of_order,
            "callback_delivery_delay_ms_p50": (
                round(_percentile(delays_ms, 0.5), 3) if valid else None),
            "callback_delivery_delay_ms_p95": (
                round(_percentile(delays_ms, 0.95), 3) if valid else None),
            "callback_delivery_delay_ms_max": (
                round(max(delays_ms), 3) if valid else None),
            "not_proof_of_dds_packet_loss": True,
            "not_physical_camera_imu_offset": True,
            "calibration_ready": False,
        }

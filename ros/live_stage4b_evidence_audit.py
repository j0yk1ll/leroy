#!/usr/bin/env python3
"""Stage 4B offline audit of a Stage 4A *text log*, with NO hardware/ROS imports.

Checks whether the bounded physical transport gate was reported PASS, then
explicitly withholds claims about DDS-loss causation, exposure/MCU time phase,
gyro/accelerometer noise and camera/IMU spatial calibration.

Run using a local log file (or "-": stdin). Never accepts arbitrary pickle
or asks the robot/MCU for data. NO motion authority.
"""
import argparse
import json
import math
import sys
from pathlib import Path

MARKERS = {
    "STAGE4A_PRE_SERIAL_DDS_GRAPH": "PASS",
    "STAGE4A_IMU_START_ACK": "PASS",
    "STAGE4A_IMU_STOP_ACK": "PASS",
    "STAGE4A_IMU_OVERLAP": "PASS_TRANSPORT_ONLY",
    "STAGE4A_PROCESS_RC": "0",
    "STAGE4A": "PASS_TRANSPORT_ONLY_NOT_VIO_OR_CALIBRATION",
}
FAULT_FIELDS = (
    "calibration_withheld", "invalid_ascii_lines", "malformed_imu_packets",
    "sequence_faults", "sequence_gap_samples", "stale_or_future_withheld",
    "timestamp_faults", "unexpected_serial_lines",
)
MAX_FILE_BYTES = 2 * 1024 * 1024
MAX_LINE_CHARS = 80_000


class EvidenceError(ValueError):
    pass


def _object_no_duplicate_keys(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise EvidenceError("duplicate_JSON_key:" + key)
        obj[key] = value
    return obj


def _nonfinite(value):
    raise EvidenceError("nonfinite_JSON:" + value)


def _require(condition, reason):
    if not condition:
        raise EvidenceError(reason)


def _int(n, label, minimum=0):
    _require(type(n) is int and n >= minimum, "bad_" + label)
    return n


def _finite(n, label, minimum=None, maximum=None):
    _require(type(n) in (int, float) and math.isfinite(n), "bad_" + label)
    if minimum is not None:
        _require(n >= minimum, "too_small_" + label)
    if maximum is not None:
        _require(n <= maximum, "too_large_" + label)
    return float(n)


def parse_text(raw):
    if isinstance(raw, bytes):
        _require(len(raw) <= MAX_FILE_BYTES, "oversized_log")
        raw = raw.decode("utf-8", "strict")
    _require(len(raw.encode("utf-8")) <= MAX_FILE_BYTES, "oversized_log")
    report = None
    seen = {}
    for line in raw.splitlines():
        _require(len(line) <= MAX_LINE_CHARS, "oversized_log_line")
        line = line.strip()
        if not line:
            continue
        # Logs may contain other lines, including Docker build output.
        for key in (*MARKERS, "STAGE4A_RESULT_JSON"):
            prefix = key + "="
            if line.startswith(prefix):
                value = line[len(prefix):]
                # A real supervised session prints this STARTING lifecycle
                # marker before the final STAGE4A=PASS/blocked result. It is
                # not a second acceptance result.
                if key == "STAGE4A" and value == "STARTING_PHYSICAL_IMU_ONLY":
                    break
                _require(key not in seen, "duplicate_marker_" + key)
                seen[key] = value
                if key == "STAGE4A_RESULT_JSON":
                    try:
                        report = json.loads(
                            value, object_pairs_hook=_object_no_duplicate_keys,
                            parse_constant=_nonfinite)
                    except (json.JSONDecodeError, TypeError) as exc:
                        raise EvidenceError("invalid_result_json") from exc
                break
    _require(report is not None and isinstance(report, dict), "missing_result")
    for key, value in MARKERS.items():
        _require(seen.get(key) == value, "stage4a_marker_not_accepted:" + key)
    return report


def audit(report):
    """Require Stage 4A semantics but NEVER promote it to physical calibration."""
    _require(isinstance(report, dict), "invalid_report")
    _require(report.get("start_ack") is True and report.get("stop_ack") is True,
             "missing_mcu_start_or_stop_ack")
    _require(report.get("error") is None, "reported_error")
    duration = _finite(report.get("duration_s"), "duration_s", 60.0, 100.0)
    overlap = _finite(report.get("overlap_receipt_s"), "overlap_s", 15.0, duration)
    counts = report.get("counts")
    src = report.get("source")
    _require(isinstance(counts, dict) and isinstance(src, dict), "missing_counters")
    src_counts = src.get("counts")
    _require(isinstance(src_counts, dict), "missing_source_counters")
    for key in FAULT_FIELDS:
        _require(_int(src_counts.get(key), key) == 0, "source_fault:" + key)
    received = _int(src_counts.get("received_imu_packets"), "received_imu_packets", 1)
    published = _int(src_counts.get("published_imu_messages"), "published_imu_messages", 1)
    _require(published == received, "source_ingress_vs_publication_discrepancy")
    observed = _int(counts.get("imu"), "imu_observed", 1)
    images = _int(counts.get("image"), "camera_image_count", 1)
    camera_info = _int(counts.get("info"), "camera_info_count", 1)
    _require(observed <= published, "observer_count_exceeds_source")
    _require(_int(counts.get("odom"), "odom") == 0, "unexpected_odometry")
    _require(_int(counts.get("tf"), "tf") == 0, "unexpected_dynamic_tf")
    _require(src.get("clock_rate_estimate_ready") is True, "clock_rate_not_ready")
    _require(src.get("terminal_fault") is None, "source_terminal_fault")
    _require(src.get("pub_topic") == "/imu/data_raw" and
             src.get("publisher_node") == "/m18_imu_source", "unexpected_imu_source")
    _require(src.get("motor_commands_sent") == 0, "unexpected_motor_commands")
    rate = _finite(src.get("measured_publication_rate_hz"),
                   "source_rate_hz", 48.0, 52.0)
    ppm = _finite(src.get("clock_rate_estimate_ppm"), "clock_rate_ppm", -200.0, 200.0)
    withheld = (
        "camera_imu_absolute_offset_validated", "camera_imu_extrinsics_validated",
        "xrslam_vio_started", "metric_odometry_authorized",
    )
    for key in withheld:
        _require(report.get(key) is False, "unsafe_claim:" + key)
    for key in ("absolute_mcu_camera_offset_validated", "spatial_imu_extrinsics_validated",
                "gyro_noise_validated", "xrslam_ready"):
        _require(src.get(key) is False, "unsafe_source_claim:" + key)
    return {
        "audit": "STAGE4B_OFFLINE_STAGE4A_EVIDENCE_VALID",
        "stage4a_transport_pass": True,
        "stage4b_calibration_ready": False,
        "stage4b_blockers": [
            "unknown_camera_exposure_to_imu_acquisition_offset",
            "unverified_camera_imu_rigid_extrinsics",
            "unverified_imu_noise_and_bias",
            "no_per_message_DDS_subscriber_matching_or_latency_trace",
            "no_validated_metric_VIO_or_pose_convention",
        ],
        "duration_s": duration,
        "receipt_overlap_s": overlap,
        "source_imu_received": received,
        "source_imu_published": published,
        "passive_imu_observed": observed,
        "observed_to_published_fraction": round(observed / published, 6),
        "not_a_DDS_packet_loss_rate": True,
        "camera_images_observed": images,
        "camera_info_observed": camera_info,
        "source_imu_rate_hz": rate,
        "host_relative_mcu_clock_rate_ppm": ppm,
        "physical_camera_imu_offset_s": None,
        "camera_imu_extrinsics_validated": False,
        "imu_noise_validated": False,
        "enable_metric_odometry": False,
        "enable_map_tf": False,
        "enable_motors": False,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log", help="existing Stage 4A console log path or '-' for stdin")
    args = ap.parse_args(argv)
    try:
        if args.log == "-":
            raw = sys.stdin.buffer.read(MAX_FILE_BYTES + 1)
        else:
            p = Path(args.log)
            if not p.is_file() or p.is_symlink():
                raise EvidenceError("log_must_be_regular_file")
            if p.stat().st_size > MAX_FILE_BYTES:
                raise EvidenceError("oversized_log")
            raw = p.read_bytes()
        result = audit(parse_text(raw))
        print("STAGE4B_READINESS_JSON=" + json.dumps(result, sort_keys=True))
        print("STAGE4B_RESULT=TRANSPORT_AUDITED_CALIBRATION_BLOCKED")
        return 0
    except (EvidenceError, OSError, UnicodeDecodeError) as exc:
        print("STAGE4B_RESULT=BLOCKED:" + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

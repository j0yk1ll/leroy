#!/usr/bin/env python3
"""M23AV: offline read-only M23AU incident log consistency audit.

Infers a repeated timestamp in RAW SUBSCRIBER observations, not the publisher.
Inert by default. No ROS, Podman, camera, network or device access.
"""
import argparse
import json
from pathlib import Path
import sys


class Blocked(ValueError):
    pass


def marker(lines, prefix):
    found = [line[len(prefix):] for line in lines if line.startswith(prefix)]
    if len(found) != 1:
        raise Blocked("missing or duplicate marker: " + prefix)
    return found[0]


def verify_log(text):
    lines = text.splitlines()
    raw = marker(lines, "M23K_CAMERA_PROBE_JSON=")
    rect = marker(lines, "M23O_RECT_QOS_JSON=")
    if raw != rect:
        raise Blocked("M23K and M23O reports differ")
    report = json.loads(raw)
    if not isinstance(report, dict):
        raise Blocked("report is not an object")
    if marker(lines, "M23K_VERDICT=") != "INCOMPLETE_CAMERA_PATH_OBSERVATION":
        raise Blocked("missing blocked M23K verdict")
    if marker(lines, "M23O_RECT_QOS_VERDICT=") != "INCOMPLETE_CAMERA_PATH_OBSERVATION":
        raise Blocked("missing blocked M23O verdict")
    if "inherited M23O rectified continuity and/or graph failed" not in marker(lines, "M23AU_BLOCKED="):
        raise Blocked("unexpected M23AU block reason")
    if marker(lines, "M23AU_PRODUCTION_ACCEPTANCE=") != "BLOCKED":
        raise Blocked("production may not be accepted")
    if report.get("faults") != ["NONMONOTONIC_RAW"]:
        raise Blocked("not exactly the observed raw timestamp anomaly")
    if report.get("shutdown_clean") is not True:
        raise Blocked("camera child process shutdown was not clean")
    if report.get("graph_qos_verified") is not True or report.get("receiver_qos_verified") is not True:
        raise Blocked("ROS graph / receiver QoS not verified")
    if report.get("actual_publisher_offered_qos_verified") is not True:
        raise Blocked("ROS publisher offered QoS not verified")
    if report.get("camera_probe_variant") != "full_mjpeg" or report.get("rectifier_enabled") is not True:
        raise Blocked("wrong camera-mode scope")
    if report.get("device_access") != "CAMERA_ONLY_BY_OPERATOR" or report.get("stm32_serial_access") is not False:
        raise Blocked("wrong device scope")
    if report.get("camera_publisher_rate_verified") is not False:
        raise Blocked("unsupported publication-side rate claim")
    if report.get("rectified_image_continuity_validated") is not False:
        raise Blocked("false rectified continuity acceptance")
    counts = report["counts"]
    if counts != {"raw": 1404, "rect": 1403, "info": 1403}:
        raise Blocked("incident frame counts changed")
    paired = report["raw_rect_identical_header_stamp_pairs"]
    raw_only = report["raw_only_observed_header_stamps"]
    rect_only = report["rect_only_observed_header_stamps"]
    if any(type(x) is not int or x < 0 for x in (paired, raw_only, rect_only)):
        raise Blocked("stamp cardinalities invalid")
    unique_raw = paired + raw_only
    unique_rect = paired + rect_only
    if unique_raw > counts["raw"] or unique_rect > counts["rect"]:
        raise Blocked("unique stamp cardinality exceeds observed messages")
    raw_repeat_excess = counts["raw"] - unique_raw
    rect_repeat_excess = counts["rect"] - unique_rect
    if (unique_raw, unique_rect, raw_repeat_excess, rect_repeat_excess) != (1403, 1403, 1, 0):
        raise Blocked("stamp set/cardinality does not prove one repeated raw observation")
    for kind in ("raw", "rect", "info"):
        stream = report["streams"][kind]
        if stream.get("count") != counts[kind]:
            raise Blocked("stream count mismatch: " + kind)
        hz = stream.get("observed_rate_hz")
        if type(hz) not in (int, float) or not 25 <= hz <= 35:
            raise Blocked("observer rate outside expected range: " + kind)
        for interval_kind in ("header_intervals", "receipt_intervals"):
            if stream[interval_kind].get("gaps_over_250ms") != 0:
                raise Blocked("large observation gap: " + kind)
        expected_nonpositive = 1 if kind == "raw" else 0
        if stream["header_intervals"].get("nonpositive_gaps") != expected_nonpositive:
            raise Blocked("unexpected header order anomaly: " + kind)
        if stream["receipt_intervals"].get("nonpositive_gaps") != 0:
            raise Blocked("unexpected receipt ordering anomaly: " + kind)
    return {
        "verdict": "ONE_REPEATED_OBSERVED_RAW_HEADER_STAMP",
        "unique_raw_header_stamps": unique_raw,
        "unique_rectified_header_stamps": unique_rect,
        "raw_observer_message_count": counts["raw"],
        "rect_observer_message_count": counts["rect"],
        "raw_repeated_header_stamp_excess": raw_repeat_excess,
        "raw_nonpositive_intervals": 1,
        "unique_rect_headers_all_seen_in_raw": True,
        "ros_graph_qos_verified": True,
        "camera_and_rectifier_shutdown_reported_clean": True,
        "camera_publisher_duplicate_proven": False,
        "dds_redelivery_or_reordering_excluded": False,
        "raw_sample_sequence_preserved_in_host_log": False,
        "live_test_acceptance": "BLOCKED",
        "physical_camera_retest_authorized": False,
        "production_acceptance": "BLOCKED",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.log is None:
        print("M23AV_STATUS=OFFLINE_AND_INERT_BY_DEFAULT", flush=True)
        return 0
    try:
        if args.log.is_symlink() or not args.log.is_file():
            raise Blocked("log must be existing regular file")
        if args.log.stat().st_size > 2_000_000:
            raise Blocked("unexpected log size")
        text = args.log.read_text(encoding="utf-8")
        report = verify_log(text)
        print("M23AV_RESULT_JSON=" + json.dumps(report, sort_keys=True), flush=True)
        print("M23AV_VERDICT=ONE_REPEATED_OBSERVED_RAW_HEADER_STAMP", flush=True)
        print("M23AV_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 0
    except (OSError, UnicodeError, ValueError, TypeError, KeyError) as exc:
        print("M23AV_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:160], flush=True)
        print("M23AV_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""M21 initial offline audit of an existing M20 text log; never touches ROS or devices.

This is a readiness/diagnostics audit, NOT an exposure-to-IMU offset estimator.
In particular, subtracting camera and IMU header ages cannot recover the
physical phase of two independently delayed sensor pipelines.
"""

import argparse
import json
from pathlib import Path
import sys

TOPICS = ("camera_image", "camera_info", "imu")
MIN_COUNT = {"camera_image": 400, "camera_info": 400, "imu": 5600}
FRAMES = {"camera_image": "camera_optical_frame",
          "camera_info": "camera_optical_frame", "imu": "imu_link"}
PUBLISHERS = {
    "camera_image": {"node": "rectify_node", "namespace": "/camera",
                     "type": "sensor_msgs/msg/Image"},
    "camera_info": {"node": "usb_cam", "namespace": "/camera",
                    "type": "sensor_msgs/msg/CameraInfo"},
    "imu": {"node": "m18_imu_source", "namespace": "/",
            "type": "sensor_msgs/msg/Imu"},
}
VERDICT = "READY_FOR_M21_CORRELATED_TIMING_EXPERIMENT_DESIGN"
INCOMPLETE = "INCOMPLETE_M20_TIMING_EVIDENCE"


def load_log(path):
    """Parse exactly one JSON report of each kind from the existing M20 capture."""
    json_prefixes = {"M16_PASSIVE_REPORT_JSON": "m16",
                     "M20_COMBINED_REPORT_JSON": "m20"}
    values = {}
    records = {}
    with Path(path).open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            line = line.strip()
            for prefix, key in json_prefixes.items():
                start = prefix + "="
                if line.startswith(start):
                    if key in records:
                        raise ValueError("duplicate " + prefix)
                    records[key] = json.loads(line[len(start):])
            for prefix in ("M20_IMU_STREAM_START_ACK", "M20_IMU_STREAM_STOP_ACK",
                           "M20_COMBINED_VERDICT", "M16_VERDICT"):
                start = prefix + "="
                if line.startswith(start):
                    if prefix in values:
                        raise ValueError("duplicate " + prefix)
                    values[prefix] = line[len(start):]
    return records, values


def audit(records, markers):
    """Summarize transport evidence, deliberately withholding physical offset."""
    faults = []
    m20 = records.get("m20", {})
    m16 = records.get("m16", {})
    topics = m16.get("topics", {})
    source = m20.get("imu_source", {})
    counts = source.get("counts", {})

    def require(condition, reason):
        if not condition:
            faults.append(reason)

    require(markers.get("M20_COMBINED_VERDICT") ==
            "OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT", "missing_accepted_m20_marker")
    require(m20.get("verdict") == "OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT",
            "m20_report_not_accepted")
    require(markers.get("M20_IMU_STREAM_START_ACK") == "PASS" and
            m20.get("imu_stream_start_acknowledged") is True, "start_ack_not_pass")
    require(markers.get("M20_IMU_STREAM_STOP_ACK") == "PASS" and
            m20.get("imu_stream_stop_acknowledged") is True, "stop_ack_not_pass")
    require(m20.get("duration_requested_s") == 120 and
            m20.get("observation_completed") is True, "observation_incomplete")
    overlap = m20.get("sensor_receipt_overlap_s")
    require(isinstance(overlap, (int, float)) and 90.0 <= overlap <= 121.0,
            "receipt_overlap_unverified")
    require(m20.get("clock_rate_ready") is True and
            source.get("clock_rate_estimate_ready") is True,
            "imu_clock_rate_mapper_not_ready")
    require(m20.get("error") is None and
            m20.get("forbidden_graph_fault") is None and
            not m20.get("observer_thread_faults"), "runtime_or_graph_fault")
    require(m20.get("motor_commands_sent") == 0 and
            source.get("motor_commands_sent") == 0, "unexpected_motor_command_report")
    require(counts.get("published_imu_messages", 0) >= 5700,
            "insufficient_imu_source_messages")
    require(markers.get("M16_VERDICT") == "INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS"
            and m16.get("verdict") == "INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS"
            and m16.get("observation_only") is True, "passive_m16_report_missing")
    diagnostics = {}
    for key in TOPICS:
        st = topics.get(key, {})
        require(st.get("accepted", 0) >= MIN_COUNT[key] and st.get("invalid") == 0
                and st.get("received") == st.get("accepted")
                and st.get("fresh_valid_receipt") is True,
                key + "_message_contract_not_observed")
        require(st.get("frame") == FRAMES[key]
                and st.get("publishers") == [PUBLISHERS[key]],
                key + "_source_or_frame_mismatch")
        diagnostics[key] = {field: st.get(field) for field in (
            "accepted", "invalid", "frame", "mean_receipt_hz",
            "last_acquisition_age_s", "min_acquisition_age_s",
            "max_acquisition_age_s", "max_receipt_gap_s")}
    result = {
        "milestone": "M21_OFFLINE_READINESS",
        "evidence_source": "existing_M20_log_only",
        "m20_receipt_overlap_s": overlap,
        "mcu_clock_rate_ppm_host_relative": m20.get("clock_rate_estimate_ppm"),
        "per_stream_ros_message_diagnostics": diagnostics,
        "physical_camera_imu_offset_s": None,
        "physical_offset_validated": False,
        "physical_extrinsics_validated": False,
        "explanation": "ROS header ages and overlap cannot establish camera exposure-to-IMU acquisition offset; McuClockSync uses an unknown one-way receipt delay.",
        "faults": faults,
        "verdict": VERDICT if not faults else INCOMPLETE,
    }
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("log", help="existing m20_real_camera_imu.log; READ ONLY")
    args = p.parse_args(argv)
    try:
        records, markers = load_log(args.log)
        result = audit(records, markers)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print("M21_OFFLINE_ERROR=" + type(exc).__name__ + ": " + str(exc),
              file=sys.stderr)
        return 2
    print("M21_OFFLINE_REPORT_JSON=" + json.dumps(result, sort_keys=True))
    print("M21_OFFLINE_VERDICT=" + result["verdict"])
    return 0 if not result["faults"] else 2


if __name__ == "__main__":
    sys.exit(main())

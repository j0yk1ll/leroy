#!/usr/bin/env python3
"""M21C offline JSONL sensor-metadata contract. NEVER opens ROS or devices.

Format readiness is not a camera/IMU physical acquisition-offset estimate.
"""
import argparse
import json
import math
from pathlib import Path
import sys

KINDS = {"camera_image", "camera_info", "imu"}
BASE = {"kind", "receipt_mono_ns", "header_ros_ns", "frame_id", "publisher"}
FIELDS = {
    "camera_image": BASE | {"frame_index", "width", "height"},
    "camera_info": BASE | {"width", "height"},
    "imu": BASE | {"mcu_seq", "mcu_t_us", "gyro_xyz_rad_s"},
}
EXPECTED = {
    "camera_image": ("camera_optical_frame", "/camera/rectify_node"),
    "camera_info": ("camera_optical_frame", "/camera/usb_cam"),
    "imu": ("imu_link", "/m18_imu_source"),
}
UINT32 = 1 << 32


def integer(value, name, low=0, high=None):
    if type(value) is not int or value < low or (high is not None and value > high):
        raise ValueError(name + ": invalid integer")
    return value


def summarize(records):
    """Reject malformed/unattributed metadata; NEVER infer camera/IMU phase."""
    if not records or not isinstance(records[0], dict):
        raise ValueError("missing first session record")
    first = records[0]
    if set(first) != {"kind", "schema", "evidence_type"} or first["kind"] != "session" or first["schema"] != "leroy_m21c_v1" or first["evidence_type"] not in {"SYNTHETIC", "REAL_UNCALIBRATED"}:
        raise ValueError("invalid session manifest")
    last = {}
    stats = {k: {"messages": 0, "max_receive_gap_s": 0.0,
                 "sequence_missing_samples": 0} for k in KINDS}
    first_time = {}
    for row in records[1:]:
        if not isinstance(row, dict) or row.get("kind") not in KINDS:
            raise ValueError("unexpected or missing event kind")
        kind = row["kind"]
        if set(row) != FIELDS[kind]:
            raise ValueError(kind + ": prohibited or missing metadata field")
        if (row["frame_id"], row["publisher"]) != EXPECTED[kind]:
            raise ValueError(kind + ": frame or publisher mismatch")
        stamp = integer(row["header_ros_ns"], kind + " header_ros_ns", 1)
        recv = integer(row["receipt_mono_ns"], kind + " receipt_mono_ns", 1)
        if kind != "imu":
            integer(row["width"], kind + " width", 1, 16000)
            integer(row["height"], kind + " height", 1, 16000)
            if row["width"] != 640 or row["height"] != 480:
                raise ValueError(kind + ": unexpected dimensions")
        if kind == "camera_image":
            integer(row["frame_index"], "frame_index", 0)
        if kind == "imu":
            integer(row["mcu_seq"], "mcu_seq", 0, UINT32 - 1)
            integer(row["mcu_t_us"], "mcu_t_us", 0, UINT32 - 1)
            gyro = row["gyro_xyz_rad_s"]
            if not isinstance(gyro, list) or len(gyro) != 3 or any(
                    type(x) not in (int, float) or not math.isfinite(x) for x in gyro):
                raise ValueError("imu: bad gyro data")
        if kind in last:
            old = last[kind]
            if stamp <= old["header_ros_ns"] or recv <= old["receipt_mono_ns"]:
                raise ValueError(kind + ": duplicate or regressed stamps")
            gap = (recv - old["receipt_mono_ns"]) / 1e9
            stats[kind]["max_receive_gap_s"] = max(stats[kind]["max_receive_gap_s"], gap)
            if kind == "camera_image" and row["frame_index"] != old["frame_index"] + 1:
                raise ValueError("camera_image: noncontiguous local frame index")
            if kind == "imu":
                delta_seq = (row["mcu_seq"] - old["mcu_seq"]) % UINT32
                delta_us = (row["mcu_t_us"] - old["mcu_t_us"]) % UINT32
                if not (1 <= delta_seq <= 1000 and 1 <= delta_us <= 2_000_000):
                    raise ValueError("imu: invalid MCU sequence or time progression")
                stats[kind]["sequence_missing_samples"] += delta_seq - 1
        else:
            first_time[kind] = recv
        stats[kind]["messages"] += 1
        last[kind] = row
    if any(s["messages"] < 2 for s in stats.values()):
        raise ValueError("each stream requires >=2 events")
    start = max(first_time.values())
    end = min(last[k]["receipt_mono_ns"] for k in KINDS)
    overlap = max(0, end - start) / 1e9
    return {
        "schema": "leroy_m21c_v1", "evidence_type": first["evidence_type"],
        "stream_diagnostics": stats, "receipt_overlap_s": round(overlap, 6),
        "large_camera_gap_over_250ms": stats["camera_image"]["max_receive_gap_s"] > 0.25,
        "physical_camera_imu_offset_s": None,
        "physical_camera_imu_offset_validated": False,
        "camera_imu_extrinsics_validated": False,
        "verdict": "M21C_OFFLINE_METADATA_FORMAT_VALID",
    }


def load(path):
    rows = []
    with Path(path).open("r", encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, 1):
            if line_no > 200000 or len(line) > 4096:
                raise ValueError("unbounded event log")
            if not line.strip():
                raise ValueError("blank event line")
            rows.append(json.loads(line))
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="existing local metadata JSONL; READ ONLY")
    args = parser.parse_args(argv)
    try:
        report = summarize(load(args.path))
    except (ValueError, TypeError, OSError, json.JSONDecodeError) as exc:
        print("M21C_FORMAT_ERROR=" + type(exc).__name__ + ": " + str(exc))
        return 2
    print("M21C_FORMAT_REPORT_JSON=" + json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())

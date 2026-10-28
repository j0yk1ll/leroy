#!/usr/bin/env python3
"""Extract only bounded Stage 3 VIO diagnostic scalars from a private log.

No devices, ROS nodes, network, writes, frame data or IMU packet replay.
An exact SHA256 is required to prevent confusing different physical runs.
"""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys

REQUIRED = [
    "STAGE3_VIO_IMU_START_ACK=PASS",
    "STAGE3_VIO_IMU_STOP_ACK=PASS",
    "STAGE3_VIO_IMU_VERDICT=PASS_DIAGNOSTIC_STREAM",
    "M23BV_STAGE3_VIO_AUDIT=PASS_REAL_CAMERA_IMU_GRAPH_NO_ODOM",
    "M23BV_STAGE3_REAL_CAMERA_READY=PASS",
    "M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=PASS",
    "STAGE3_VIO_ODOM_MESSAGES=0",
    "STAGE3_VIO_PROVISIONAL_OFFSET_SEC=0.0_UNCALIBRATED",
    "STAGE3_VIO_URDF_TF=PROVISIONAL",
    "STAGE3_VIO_IMU_PROCESS_RC=0",
    "STAGE3_VIO_AUDIT_PROCESS_RC=0",
    "STAGE3_REAL_CAMERA_NAV2_controller_server=unconfigured",
    "STAGE3_REAL_CAMERA_NAV2_planner_server=unconfigured",
    "STAGE3_REAL_CAMERA_NAV2_bt_navigator=unconfigured",
]
SOURCE_ZERO = [
    "calibration_withheld", "malformed_imu_packets",
    "sequence_gap_samples", "sequence_faults", "timestamp_faults",
    "stale_or_future_withheld", "unexpected_serial_lines",
    "invalid_ascii_lines",
]


def blocked(why):
    print("STAGE3_VIO_LOG_REPORT=BLOCKED_" + why, file=sys.stderr)
    return 2


def review(path, sha):
    try:
        st = path.lstat()
        parent = path.parent.stat()
        if (not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode)
                or st.st_uid != os.getuid() or not 0 < st.st_size <= 32 * 1024**2
                or stat.S_IMODE(st.st_mode) & 0o077
                or stat.S_IMODE(parent.st_mode) & 0o077):
            raise ValueError("PRIVATE_FILE_PERMISSIONS_OR_SIZE")
        buf = path.read_bytes()
        actual = hashlib.sha256(buf).hexdigest()
        if actual != sha:
            raise ValueError("SHA256_MISMATCH")
        lines = buf.decode("utf-8", errors="replace").splitlines()
        if any(len(line) > 131072 for line in lines):
            raise ValueError("OVERSIZE_LOG_LINE")
        missing = set(REQUIRED) - set(lines)
        if missing:
            raise ValueError("MISSING_MARKERS:" + ",".join(sorted(missing)))
        text = "\n".join(lines)
        if any(x in text for x in (
                "STAGE3_VIO_IMU_STOP_ACK=UNVERIFIED",
                "STAGE3_VIO_MCU_STATE=UNKNOWN",
                "STAGE3_VIO_IMU_ERROR=",
                "STAGE3_PREMATURE_PROCESS_EXIT=")):
            raise ValueError("REPORTED_FAULT")
        def one(prefix):
            matches = [line[len(prefix):] for line in lines
                       if line.startswith(prefix)]
            if len(matches) != 1:
                raise ValueError("MISSING_OR_DUPLICATED:" + prefix)
            return matches[0]
        stats = json.loads(one("STAGE3_VIO_IMU_METRICS="))
        counts = stats["counts"]
        if (stats.get("terminal_fault") is not None
                or stats.get("motor_commands_sent") != 0
                or stats.get("absolute_mcu_camera_offset_validated") is not False
                or stats.get("pub_topic") != "/imu/data_raw"
                or stats.get("publisher_node") != "/m18_imu_source"
                or any(counts.get(key) != 0 for key in SOURCE_ZERO)):
            raise ValueError("IMU_SOURCE_CONTRACT")
        imu_published = counts["published_imu_messages"]
        if not isinstance(imu_published, int) or imu_published < 2000:
            raise ValueError("INSUFFICIENT_IMU")
        frame_values = [
            int(m.group(1)) for line in lines
            if (m := re.search(
                r"Submitted ([0-9]+) camera frames to XRSLAM \(monocular_imu\)",
                line))
        ]
        if not frame_values:
            raise ValueError("NO_NATIVE_XRSLAM_CAMERA_SUBMISSION")
        xr_state = one("STAGE3_VIO_XRSLAM_STATUS=")
        if xr_state not in ("initializing", "pose_convention_unverified"):
            raise ValueError("UNEXPECTED_XRSLAM_STATUS")
        auditor_imu = int(one("STAGE3_VIO_REAL_IMU_COUNT="))
        triple = int(one("STAGE3_REAL_CAMERA_MATCHED_TRIPLE="))
        depth = int(one("STAGE3_REAL_CAMERA_MATCHED_DEPTH="))
        camera_counts = ast.literal_eval(
            one("STAGE3_REAL_CAMERA_COUNTS="))
        if (auditor_imu < 200 or triple < 90 or depth < 1
                or not isinstance(camera_counts, dict)
                or any(not isinstance(camera_counts.get(k), int)
                       or camera_counts[k] < 1
                       for k in ("raw", "info", "rect", "depth"))):
            raise ValueError("INSUFFICIENT_INPUT_DATA")
        print("STAGE3_VIO_LOG_REPORT=PASS_SCALAR_EVIDENCE")
        print("STAGE3_VIO_REPORT_SHA256=" + actual)
        print("STAGE3_VIO_REPORT_NATIVE_XRSLAM_CAMERA_FRAMES_AT_LEAST="
              + str(max(frame_values)))
        print("STAGE3_VIO_REPORT_IMU_PUBLISHED=" + str(imu_published))
        print("STAGE3_VIO_REPORT_IMU_AUDITOR_RECEIVED=" + str(auditor_imu))
        for label, field in (
            ("IMU_RATE_HZ", "measured_publication_rate_hz"),
            ("IMU_CLOCK_RATE_PPM", "clock_rate_estimate_ppm"),
            ("IMU_CLOCK_RATE_READY", "clock_rate_estimate_ready"),
            ("LAST_MAPPED_AGE_ESTIMATE_S", "last_estimated_acquisition_age_s"),
            ("MAX_MAPPED_AGE_ESTIMATE_S", "max_estimated_acquisition_age_s"),
        ):
            print("STAGE3_VIO_REPORT_" + label + "="
                  + json.dumps(stats.get(field)))
        print("STAGE3_VIO_REPORT_CAMERA_COUNTS=" +
              json.dumps(camera_counts, sort_keys=True))
        print("STAGE3_VIO_REPORT_CAMERA_PAIRED=" + str(triple))
        print("STAGE3_VIO_REPORT_DEPTH_MATCHED=" + str(depth))
        print("STAGE3_VIO_REPORT_XRSLAM_STATUS=" + xr_state)
        print("STAGE3_VIO_REPORT_MCU_STOP_ACK=PASS")
        print("STAGE3_VIO_REPORT_METRIC_ODOM=NOT_VALIDATED")
        print("STAGE3_VIO_REPORT_CAMERA_IMU_PHASE=NOT_MEASURED")
        print("STAGE3_VIO_REPORT_NAV2=UNCONFIGURED")
        print("STAGE3_VIO_REPORT_HARDWARE_ACCESS=NONE")
    except (OSError, ValueError, SyntaxError, KeyError,
            TypeError, json.JSONDecodeError) as exc:
        return blocked(type(exc).__name__ + ":" + str(exc))
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--expect-sha256", required=True)
    args = parser.parse_args()
    if not re.fullmatch("[0-9a-f]{64}", args.expect_sha256):
        return blocked("BAD_SHA256_ARGUMENT")
    return review(args.log, args.expect_sha256)


if __name__ == "__main__":
    sys.exit(main())

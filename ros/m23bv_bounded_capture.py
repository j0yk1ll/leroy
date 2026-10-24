#!/usr/bin/env python3
"""Additive M23BV physical CAPTURE-CANDIDATE orchestrator, never self-authorized.

This is an unexecuted future operator path; it retains M23H ordered
sensor-parent SIGTERM and recorder process-group SIGINT finalization.
Its closed output remains PENDING offline M23BV derivation + M23I/M23A audit.
No host hardware launcher is provided by this preparation package.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

from m23g_live_recording_preflight import preflight, TOPICS
from m23h_bounded_capture import (
    _signal_and_wait, check_devices, closed_sqlite_counts, assess_counts,
    SENSOR_PROCESS_TIMEOUT_S, SENSOR_GRACE_S, ROSBAG_FLUSH_TIMEOUT_S,
    RECORD_START_SETTLE_S,
)

EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23BV_OPERATOR_CAPTURE_APPROVED": "YES",
}
QOS_PATH = "/opt/m23bv/m23bv_sensor_qos.yaml"
SENSOR_RUN = "/usr/local/bin/leroy-m23bv-sensor-run"


def recorder_argv(target):
    return ["ros2", "bag", "record", "--storage", "sqlite3",
            "--output", str(target), "--qos-profile-overrides-path", QOS_PATH,
            "--topics", *TOPICS]


def capture(root, session_id, *, spawn=subprocess.Popen, pause=time.sleep):
    if any(os.environ.get(k) != v for k, v in EXPECTED_ENV.items()):
        raise ValueError("M23BV approval/environment not established")
    check_devices()       # stat only, no open. Hardware safety is external.
    budget = preflight(root, session_id, duration_seconds=120)
    if not budget["disk_capacity_sufficient"]:
        raise ValueError("insufficient disk budget")
    target = Path(budget["output_directory"])
    log_file = target.parent / ("m23bv_capture_" + session_id + ".log")
    recorder = sensor = None
    recorder_rc = sensor_rc = None
    recorder_closed = False
    error = None
    with log_file.open("xb") as log:
        try:
            recorder = spawn(recorder_argv(target), stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
            pause(RECORD_START_SETTLE_S)
            if recorder.poll() is not None:
                raise RuntimeError("recorder exited before sensor launch")
            sensor = spawn([SENSOR_RUN], stdout=log,
                           stderr=subprocess.STDOUT, start_new_session=True)
            sensor_rc = sensor.wait(timeout=SENSOR_PROCESS_TIMEOUT_S)
        except (OSError, RuntimeError, subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            error = type(exc).__name__ + ":" + str(exc)
        finally:
            # CRITICAL: never signal whole sensor process group. M20 runner
            # must attempt STM32 STOP/ACK through its protected cleanup.
            if sensor is not None:
                rc, stopped = _signal_and_wait(sensor, signal.SIGTERM, SENSOR_GRACE_S)
                if sensor_rc is None:
                    sensor_rc = rc
                if not stopped:
                    error = (error or "") + ";M20_NO_CLEAN_SHUTDOWN"
            # SIGINT flushes bag metadata; do not SIGKILL the recorder.
            if recorder is not None:
                recorder_rc, recorder_closed = _signal_and_wait(
                    recorder, signal.SIGINT, ROSBAG_FLUSH_TIMEOUT_S,
                    process_group=True)
                if not recorder_closed:
                    error = (error or "") + ";ROSBAG_FINALIZATION_UNVERIFIED"
            log.flush()
            os.fsync(log.fileno())
    counts = {}
    audit_error = None
    if sensor_rc == 0 and recorder_rc == 0 and recorder_closed:
        try:
            counts = closed_sqlite_counts(target)
        except (OSError, ValueError, sqlite3.Error) as exc:
            audit_error = type(exc).__name__ + ":" + str(exc)
    okay = (error is None and audit_error is None and sensor_rc == 0
            and recorder_rc == 0 and recorder_closed and assess_counts(counts))
    report = {
        "milestone": "M23BV", "verdict": (
            "CLOSED_CANDIDATE_PENDING_WINDOW_DERIVATION_AND_M23A"
            if okay else "BLOCKED_CAPTURE_OR_FINALIZATION"),
        "sensor_exit_code": sensor_rc, "recorder_exit_code": recorder_rc,
        "recorder_finalized": recorder_closed, "counts": counts,
        "error": error, "audit_error": audit_error,
        "bag_directory": str(target), "capture_log": str(log_file),
        "imu_full_image_window_covered": False,  # not audited yet
        "xrslam_input_ready": False, "physical_motion_authorized": False,
    }
    print("M23BV_CAPTURE_JSON=" + json.dumps(report, sort_keys=True), flush=True)
    print("M23BV_CAPTURE_VERDICT=" + report["verdict"], flush=True)
    return 0 if okay else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output-root", type=Path, required=True)
    ap.add_argument("--session-id", required=True)
    args = ap.parse_args()
    try:
        return capture(args.output_root, args.session_id)
    except (OSError, ValueError, TypeError) as exc:
        print("M23BV_PRECONDITION=" + type(exc).__name__ + ":" + str(exc),
              file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())

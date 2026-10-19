#!/usr/bin/env python3
"""M23H: bounded real three-topic rosbag capture, operator-invoked only.

Executes the existing M20 *sensor-only* runner alongside Jazzy rosbag2.
No bridge, Nav2, XRSLAM, /cmd_vel or /odom/TF. The operator controls the
container's exact hardware device mappings and physical safe state.
Opening the MCU CDC itself may reset firmware and energize actuators.
This file NEVER establishes that physical isolation has been verified.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import stat
import subprocess
import sys
import time
from urllib.parse import quote

from m23g_live_recording_preflight import preflight, recorder_argv, TOPICS

EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23H_OPERATOR_CAPTURE_APPROVED": "YES",
}
MIN_COUNTS = {"/camera/image_rect": 400,
              "/camera/camera_info": 400,
              "/imu/data_raw": 5600}
RECORD_START_SETTLE_S = 3.0
SENSOR_PROCESS_TIMEOUT_S = 175.0
SENSOR_GRACE_S = 24.0
ROSBAG_FLUSH_TIMEOUT_S = 50.0


def check_environment(environ):
    """Software opt-in is not evidence of physical power isolation."""
    missing = [k for k, v in EXPECTED_ENV.items() if environ.get(k) != v]
    if missing:
        raise ValueError("capture environment mismatch: " + ",".join(missing))


def check_devices():
    """Checking stat does not open either physical data device."""
    for name in ("/dev/ttyACM0", "/dev/video2"):
        if not stat.S_ISCHR(os.stat(name).st_mode):
            raise ValueError(name + " must be mapped character device")
    if not Path("/config/camera/front_camera.yaml").is_file():
        raise ValueError("M20 camera calibration file absent")


def _signal_and_wait(proc, signum, grace, *, process_group=False):
    if proc is None:
        return None, False
    if proc.poll() is not None:
        return proc.returncode, True
    try:
        if process_group:
            os.killpg(proc.pid, signum)
        else:
            proc.send_signal(signum)
    except ProcessLookupError:
        pass
    try:
        return proc.wait(timeout=grace), True
    except subprocess.TimeoutExpired:
        return None, False


def closed_sqlite_counts(folder):
    """Check a finalized recorder DB *read-only*; M23E does deeper audit."""
    if not folder.is_dir() or folder.is_symlink():
        raise ValueError("final recording directory missing")
    manifest = folder / "metadata.yaml"
    if not manifest.is_file() or manifest.is_symlink():
        raise ValueError("rosbag metadata.yaml missing after shutdown")
    import yaml
    meta = yaml.safe_load(manifest.read_text("utf8"))
    info = meta.get("rosbag2_bagfile_information", {})
    if info.get("storage_identifier") != "sqlite3":
        raise ValueError("sqlite3 storage metadata required")
    files = info.get("relative_file_paths", [])
    if not isinstance(files, list) or not files or len(files) > 64:
        raise ValueError("unexpected storage file manifest")
    counts = {t: 0 for t in TOPICS}
    for name in files:
        if (type(name) is not str or Path(name).name != name
                or not name.endswith(".db3") or name in (".", "..")
                or "\\" in name or "/" in name):
            raise ValueError("unsafe rosbag file reference")
        path = folder / name
        if path.is_symlink() or not path.is_file():
            raise ValueError("missing or symlinked bag file")
        uri = "file:" + quote(str(path), safe="/") + "?mode=ro"
        connection = sqlite3.connect(uri, uri=True, timeout=2.0)
        try:
            rows = connection.execute(
                "SELECT topics.name, count(*) "
                "FROM messages INNER JOIN topics ON messages.topic_id=topics.id "
                "GROUP BY topics.name").fetchall()
            integrity = connection.execute("PRAGMA quick_check").fetchone()
            if integrity is None or integrity[0] != "ok":
                raise ValueError("sqlite quick_check failed")
            for topic, n in rows:
                if topic not in counts:
                    raise ValueError("unexpected recorded topic: " + str(topic))
                counts[topic] += int(n)
        finally:
            connection.close()
    return counts


def assess_counts(counts):
    """Frame counts are necessary, never sufficient for metric VIO."""
    return (set(counts) == set(MIN_COUNTS)
            and all(type(counts[t]) is int
                    and counts[t] >= MIN_COUNTS[t] for t in MIN_COUNTS))


def capture(root, session_id, *, spawn=subprocess.Popen,
            wait_start=time.sleep):
    check_environment(os.environ)
    check_devices()
    budget = preflight(root, session_id, duration_seconds=120)
    if not budget["disk_capacity_sufficient"]:
        raise ValueError("insufficient disk budget")
    target = Path(budget["output_directory"])
    log_file = target.parent / ("m23h_capture_" + session_id + ".log")
    # Exclusive log creation prevents silent overwrite. Recorder must create
    # its own nonexistent output directory; we never mkdir(target).
    recorder = sensor = None
    recorder_rc = sensor_rc = None
    recorder_finalized = sensor_exited = False
    run_error = None
    with log_file.open("xb") as stream:
        try:
            cmd = recorder_argv(target)
            recorder = spawn(cmd, stdout=stream, stderr=subprocess.STDOUT,
                             start_new_session=True)
            wait_start(RECORD_START_SETTLE_S)
            if recorder.poll() is not None:
                raise RuntimeError("rosbag recorder exited before sensors")
            print("M23H_RECORDER_LAUNCHED=PASS", flush=True)
            sensor = spawn(["/usr/local/bin/leroy-m20-sensor-run"],
                           stdout=stream, stderr=subprocess.STDOUT,
                           start_new_session=True)
            try:
                sensor_rc = sensor.wait(timeout=SENSOR_PROCESS_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                raise RuntimeError("M20 120-second capture deadline exceeded")
            sensor_exited = sensor_rc == 0
        except (OSError, RuntimeError, KeyboardInterrupt) as exc:
            run_error = type(exc).__name__ + ": " + str(exc)
        finally:
            # SIGTERM ONLY to M20 parent shell; its trap should shut down
            # IMU first and attempt verified stream STOP, then camera.
            # Signalling the whole group would skip the ordered cleanup.
            if sensor is not None:
                rc, stopped = _signal_and_wait(
                    sensor, signal.SIGTERM, SENSOR_GRACE_S)
                if sensor_rc is None:
                    sensor_rc = rc
                if not stopped:
                    run_error = (run_error or "") + "; M20_NO_CLEAN_SHUTDOWN"
            # ROS 2 bag record SIGINT requests flush/metadata finalization.
            # Never SIGKILL a recorder to produce a fraudulent valid bag.
            if recorder is not None:
                recorder_rc, recorder_finalized = _signal_and_wait(
                    recorder, signal.SIGINT, ROSBAG_FLUSH_TIMEOUT_S,
                    process_group=True)
                if not recorder_finalized:
                    run_error = (run_error or "") + "; BAG_FLUSH_UNVERIFIED"
            stream.flush()
            os.fsync(stream.fileno())
    counts = {}
    audit_error = None
    if sensor_rc == 0 and recorder_rc == 0 and recorder_finalized:
        try:
            counts = closed_sqlite_counts(target)
        except (ValueError, OSError, sqlite3.Error, TypeError,
                AttributeError, KeyError) as exc:
            audit_error = type(exc).__name__ + ": " + str(exc)
    complete = (run_error is None and audit_error is None
                and sensor_rc == 0 and sensor_exited
                and recorder_rc == 0 and recorder_finalized
                and assess_counts(counts))
    report = {
        "milestone": "M23H",
        "verdict": ("CLOSED_SENSOR_BAG_RECORDED_PENDING_M23F"
                    if complete else "BLOCKED_CAPTURE_OR_FINALIZATION"),
        "bag_directory": str(target),
        "log_file": str(log_file),
        "sensor_exit_code": sensor_rc,
        "recorder_exit_code": recorder_rc,
        "recorder_shutdown_completed": recorder_finalized,
        "topic_message_counts": counts,
        "minimum_counts_met": assess_counts(counts),
        "error": run_error,
        "audit_error": audit_error,
        "physical_capture_claimed_by_operator": True,
        "file_bytes_authenticated": False,
        "physical_camera_imu_phase_calibrated": False,
        "spatial_extrinsics_calibrated": False,
        "real_xrslam_tracking_validated": False,
        "metric_pose_authorized": False,
        "physical_motion_authorized": False,
    }
    print("M23H_CAPTURE_JSON=" + json.dumps(report, sort_keys=True), flush=True)
    print("M23H_VERDICT=" + report["verdict"], flush=True)
    print("M23H_XRSLAM_TRACKING_VALIDATED=FALSE", flush=True)
    print("M23H_CAMERA_IMU_CALIBRATION_VALIDATED=FALSE", flush=True)
    if not recorder_finalized:
        print("M23H_RECORDER_FINALIZATION=UNVERIFIED", flush=True)
    return 0 if complete else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    args = parser.parse_args(argv)
    try:
        return capture(args.output_root, args.session_id)
    except (OSError, TypeError, ValueError) as exc:
        print("M23H_PRECONDITION=" + type(exc).__name__ +
              ": " + str(exc), file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())

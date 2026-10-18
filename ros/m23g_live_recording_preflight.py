#!/usr/bin/env python3
"""M23G: non-actuating, offline preparation for a FUTURE real sensor bag.

NO hardware is opened, ROS is not imported, no recording starts.
The output is a command PLAN only, never an authorization or a launcher.
Actual access to the STM32 CDC can reset it and energize real actuators.
Physical motor-power isolation and explicit renewed authorization are
independent gates outside this software preflight.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import sys

CAMERA_HZ_BUDGET = 30
WIDTH, HEIGHT, BYTES_PER_PIXEL = 640, 480, 3
MIN_SECONDS, MAX_SECONDS = 30, 180
SAFETY_MULTIPLIER = 2
RESERVE_BYTES = 2 * 1024**3
QOS_PATH = "/opt/m23g/m23g_sensor_qos.yaml"
TOPICS = ("/camera/image_rect", "/camera/camera_info",
          "/imu/data_raw")
PROHIBITED_PREFIXES = ("/dev", "/proc", "/sys", "/run")
SESSION_ID = re.compile(r"[a-z0-9][a-z0-9_-]{0,47}\Z")


def required_bytes(duration_seconds):
    """Pessimistic uncompressed RGB payload and DB overhead headroom."""
    if type(duration_seconds) is not int:
        raise TypeError("duration must be an integer")
    if not MIN_SECONDS <= duration_seconds <= MAX_SECONDS:
        raise ValueError("unsupported bounded acquisition duration")
    image_bytes = (CAMERA_HZ_BUDGET * WIDTH * HEIGHT * BYTES_PER_PIXEL
                   * duration_seconds)
    return image_bytes * SAFETY_MULTIPLIER + RESERVE_BYTES


def validate_root(root):
    if not isinstance(root, Path):
        raise TypeError("explicit output parent must be a Path")
    if root.is_symlink() or not root.is_dir():
        raise ValueError("output parent must be an existing real directory")
    resolved = root.resolve(strict=True)
    text = os.fspath(resolved)
    if text == "/" or any(
            text == p or text.startswith(p + "/")
            for p in PROHIBITED_PREFIXES):
        raise ValueError("system/device directory is forbidden")
    if not os.access(root, os.W_OK | os.X_OK):
        raise ValueError("output parent is not writable/traversable")
    return resolved


def recorder_argv(destination):
    """Declarative ONLY. NEVER call subprocess or execute this argument list."""
    return [
        "ros2", "bag", "record",
        "--storage", "sqlite3",
        "--output", str(destination),
        "--qos-profile-overrides-path", QOS_PATH,
        "--topics", *TOPICS,
    ]


def preflight(root, session_id, *, duration_seconds=120):
    """Inspect local disk capacity without writing/launching anything."""
    if not isinstance(session_id, str) or not SESSION_ID.fullmatch(session_id):
        raise ValueError("unsafe session ID")
    root = validate_root(root)
    target = root / ("m23g_live_" + session_id)
    if target.exists() or target.is_symlink():
        raise ValueError("refuse to overwrite existing recording target")
    needed = required_bytes(duration_seconds)
    stat = os.statvfs(root)
    free = stat.f_bavail * stat.f_frsize
    enough = free >= needed
    return {
        "milestone": "M23G",
        "software_preflight_verdict": (
            "DISK_BUDGET_PASS_HARDWARE_NOT_AUTHORIZED" if enough
            else "BLOCKED_INSUFFICIENT_CAPTURE_SPACE"),
        "output_directory": str(target),
        "planned_duration_seconds": duration_seconds,
        "camera_nominal_budget_hz": CAMERA_HZ_BUDGET,
        "max_uncompressed_image_bytes": (
            CAMERA_HZ_BUDGET * WIDTH * HEIGHT * BYTES_PER_PIXEL
            * duration_seconds),
        "free_bytes_observed": free,
        "required_bytes_with_margin": needed,
        "approved_recording_topics": list(TOPICS),
        "planned_argv_NOT_EXECUTED": recorder_argv(target),
        "planned_qos_path": QOS_PATH,
        "disk_capacity_sufficient": enough,
        "physical_safety_signoff_received": False,
        "mcu_serial_access_authorized": False,
        "camera_device_access_authorized": False,
        "rosbag_recording_started": False,
        "complete_real_bag_recorded": False,
        "camera_imu_phase_calibrated": False,
        "wheel_manipulator_power_inhibited_verified": False,
        "physical_estop_verified": False,
        "mcu_device_identity_revalidated": False,
        "camera_device_identity_revalidated": False,
        "stop_ack_observed": False,
        "physical_motion_authorized": False,
        "device_access": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True,
                        help="existing offline writable directory; no files created")
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--duration-seconds", type=int, default=120)
    args = parser.parse_args(argv)
    try:
        report = preflight(args.output_root, args.session_id,
                           duration_seconds=args.duration_seconds)
    except (OSError, ValueError, TypeError) as exc:
        print("M23G_OFFLINE_PRECONDITION=" +
              type(exc).__name__ + ": " + str(exc),
              file=sys.stderr, flush=True)
        return 2
    print("M23G_PREFLIGHT_JSON=" + json.dumps(report, sort_keys=True),
          flush=True)
    print("M23G_OFFLINE_VERDICT=" + report["software_preflight_verdict"],
          flush=True)
    print("M23G_PHYSICAL_CAPTURE_AUTHORIZED=FALSE", flush=True)
    print("M23G_HARDWARE_ACCESS=NONE", flush=True)
    return 0 if report["disk_capacity_sufficient"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
